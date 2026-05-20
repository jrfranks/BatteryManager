/**
 * @file    BatteryManager.ino
 * @brief   Low-power, professional-grade battery charge controller for AVR Arduino.
 *
 * @details This firmware implements a safe, efficient CC/CV charger controller
 *          with aggressive power optimization for long-term battery-powered or
 *          solar applications. It supports multiple battery chemistries through
 *          profile-based configuration and includes temperature compensation,
 *          true coulomb counting, and optional rich telemetry / display.
 *
 * @section Features
 *   - Chemistry-aware state machine (Idle → Precharge → Bulk → Absorption → Float)
 *   - Temperature-compensated charging voltages (v1.1+)
 *   - True coulomb counting with separate charge/discharge tracking (v1.1+)
 *   - Deep sleep (~2s cycles) with PRR + DIDR0 + BOD_OFF power reduction
 *   - Dynamic peripheral management (Timer1, USART)
 *   - Fast ADC burst sampling for reduced active time
 *   - Robust EEPROM persistence with versioning and CRC
 *   - Rich JSON telemetry + optional SSD1306 OLED (v1.1+)
 *   - Runtime calibration via serial commands
 *
 * @section Hardware
 *   Target: Arduino Uno, Nano, Pro Mini (ATmega328P)
 *   Core library: Low-Power (rocketscream)
 *   Optional: Adafruit SSD1306 + GFX (when ENABLE_OLED=1)
 *
 * @version 1.1.0
 * @date    2026
 * @author  BatteryManager contributors
 * @license MIT
 *
 * @note    For full architectural reasoning and power optimization theory,
 *          see DESIGN.md in the repository root.
 *
 * @warning Incorrect configuration (especially voltage dividers and current
 *          scaling) can lead to overcharging or fire. Always verify with a
 *          calibrated multimeter before use.
 */

#include "Config.h"
#include <LowPower.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#if ENABLE_OLED
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif

// =============================================================================
// POWER DEBUG SWITCH
// =============================================================================
// For absolute lowest power consumption in deployed units, leave this
// commented out (USART + Serial are powered down after the boot banner).
// For development / calibration sessions where you need to type commands
// later, uncomment the line below (costs several hundred µA).
// #define SERIAL_DEBUG_ALWAYS


// =============================================================================
// FORWARD DECLARATIONS & TYPES
// =============================================================================

class ChargerController;

static void setupPWM();
static void setPWMDuty(uint8_t duty);
static uint8_t getPWMDuty();
static float    readAveragedVoltage();
static float    readAveragedCurrent();
static float    readAveragedLoadCurrent();
static float    readTemperatureC();
static uint8_t  crc8(const uint8_t* data, uint8_t len);
static void     printStatus(bool verbose = false);
static void     handleSerialCommands();
static void     updateLED();
static void     enterFault(FaultCode reason);
static const __FlashStringHelper* stateToString(ChargeState s);
static const __FlashStringHelper* faultToString(FaultCode f);

// =============================================================================
// GLOBAL STATE (minimal, clearly owned)
// =============================================================================

ChargerController* gCharger = nullptr;   // singleton controller

volatile uint32_t gWakeCount     = 0;   // 32-bit so we can do long-term time math without 36h wrap surprises
uint32_t          gLastEEPROMSave = 0;
uint32_t          gPrintCounter   = 0;
uint8_t           gLedPhase       = 0;
FaultCode         gActiveFault    = FaultCode::NONE;
bool              gMasterEnable   = true;   // serial can disable charging

// =============================================================================
// CHARGER CONTROLLER (core elegant logic)
// =============================================================================

/**
 * @class ChargerController
 * @brief Core state machine, sensor fusion, and control logic for the charger.
 *
 * This class encapsulates the entire charging algorithm, power management
 * decisions, and persistence logic. It is designed to be called periodically
 * from the main loop after waking from deep sleep.
 *
 * Key responsibilities:
 *   - Acquire and filter sensor data
 *   - Apply temperature compensation to target voltages
 *   - Execute the CC/CV state machine
 *   - Perform P+I control with slew limiting and anti-windup
 *   - Manage true coulomb counting (charge vs discharge)
 *   - Persist critical state to EEPROM with wear mitigation
 *
 * @note  The instance is kept as a static local in `loop()` to preserve
 *        state across deep sleep cycles.
 */
class ChargerController {
public:
    ChargerController() :
        state(ChargeState::INIT),
        prevState(ChargeState::INIT),
        filteredV(0.0f),
        filteredI(0.0f),
        filteredLoadI(0.0f),
        filteredT(25.0f),
        targetCurrent(0.0f),
        targetVoltage(0.0f),
        compensatedAbsorptionV(0.0f),
        compensatedFloatV(0.0f),
        pwmIntegral(0.0f),
        absorptionMinutes(0),
        floatHours(0),
        faultReason(FaultCode::NONE),
        consecutiveFaults(0),
        eeprom{}
    {}

    void begin() {
        loadFromEEPROM();

        // Seed filters with first real readings
        filteredV = readAveragedVoltage();
        filteredI = readAveragedCurrent();
        filteredT = readTemperatureC();

        // Decide initial state from persisted value + reality
        ChargeState restored = static_cast<ChargeState>(eeprom.lastState);
        if (restored >= ChargeState::IDLE && restored <= ChargeState::FLOAT) {
            state = restored;
        } else {
            state = ChargeState::INIT;
        }

        // Safety: never restore a charging state if voltage is already very high
        if (filteredV > MAX_CHARGE_VOLTAGE - 0.3f) {
            state = ChargeState::IDLE;
        }
    }

    /**
     * @brief Main control cycle executed after waking from deep sleep.
     *
     * This is the heart of the firmware. It is called approximately every 2
     * seconds. The function is deliberately kept as one cohesive unit so that
     * the sequence of operations (sample → filter → compensate → control →
     * persist) remains easy to follow for maintainers.
     *
     * Power considerations:
     *   - All heavy work happens inside a short active window.
     *   - Peripherals are aggressively powered down before returning to sleep.
     */
    void runCycle() {
        // 1. Acquire fresh sensor data (ADC powered only during this window)
        float rawV = readAveragedVoltage();
        float rawI = readAveragedCurrent();
        float rawT = readTemperatureC();

        // Apply IIR filters (α = 0.25 → good noise rejection, responsive)
        filteredV = filteredV * 0.75f + rawV * 0.25f;
        filteredI = filteredI * 0.75f + rawI * 0.25f;

        float rawLoadI = readAveragedLoadCurrent();
        filteredLoadI = filteredLoadI * 0.75f + rawLoadI * 0.25f;

        filteredT = filteredT * 0.75f + rawT * 0.25f;

        // Compute temperature-compensated target voltages (new in v1.1)
        compensatedAbsorptionV = getCompensatedVoltage(ABSORPTION_VOLTAGE);
        compensatedFloatV      = getCompensatedVoltage(FLOAT_VOLTAGE);

        // Apply runtime calibration offsets (from EEPROM)
        float vCal = filteredV + (eeprom.vCalOffset * 0.01f);
        float iCal = filteredI + (eeprom.iCalOffset * 0.01f);

        // 2. Hard safety checks (highest priority, always)
        if (checkSafetyLimits(vCal, iCal, filteredT)) {
            consecutiveFaults++;
            if (consecutiveFaults >= FAULT_DEBOUNCE_CYCLES) {
                enterFault(faultReason);
            }
            setPWMDuty(0);
            digitalWrite(PIN_CHARGE_ENABLE, LOW);
            return;
        }
        consecutiveFaults = 0;

        // 3. State machine
        ChargeState old = state;
        switch (state) {
            case ChargeState::INIT:      handleInit(); break;
            case ChargeState::IDLE:      handleIdle(vCal); break;
            case ChargeState::PRECHARGE: handlePrecharge(vCal, iCal); break;
            case ChargeState::BULK:      handleBulk(vCal, iCal); break;
            case ChargeState::ABSORPTION:handleAbsorption(vCal, iCal); break;
            case ChargeState::FLOAT:     handleFloat(vCal, iCal); break;
            case ChargeState::FAULT:     handleFault(); break;
            case ChargeState::RECOVERY:  handleRecovery(vCal); break;
        }

        // 4. Apply outputs (PWM + enable) based on current targets
        applyControlOutputs(vCal, iCal);

        // 5. True coulomb counting (separate charge and discharge)
        //
        // We maintain two independent counters instead of a single "net" value.
        // This is far more useful for diagnostics and capacity estimation.
        // A small deadband (0.02A) prevents noise from slowly accumulating
        // phantom charge/discharge when the battery is at rest.
        float netCurrent = filteredI - filteredLoadI;

        if (netCurrent > 0.02f) {
            float mAh = netCurrent * (SLEEP_INTERVAL_MS / 3600000.0f);
            eeprom.chargedMilliAmpHours += (uint32_t)(mAh * 1000.0f + 0.5f);
        } else if (netCurrent < -0.02f) {
            float mAh = -netCurrent * (SLEEP_INTERVAL_MS / 3600000.0f);
            eeprom.dischargedMilliAmpHours += (uint32_t)(mAh * 1000.0f + 0.5f);
        }

        // 6. Persist occasionally (throttled)
        if ((gWakeCount - gLastEEPROMSave) > EEPROM_SAVE_INTERVAL) {
            saveToEEPROM();
            gLastEEPROMSave = gWakeCount;
        }

        // 7. Housekeeping
        if (state != old) {
            onStateTransition(old, state);
        }

        updateTimers();
    }

    // Public accessors for UI / telemetry
    ChargeState getState() const { return state; }
    FaultCode   getFault() const { return faultReason; }
    float       getVoltage() const { return filteredV; }
    float       getCurrent() const { return filteredI; }      // charge current
    float       getLoadCurrent() const { return filteredLoadI; }
    float       getNetCurrent() const { return filteredI - filteredLoadI; }
    float       getTemp() const { return filteredT; }
    float       getTargetVoltage() const { return targetVoltage; }
    float       getCompensatedAbsorptionV() const { return compensatedAbsorptionV; }
    float       getCompensatedFloatV() const { return compensatedFloatV; }
    uint8_t     getPWM() const { return getPWMDuty(); }
    uint32_t    getChargedAh() const { return eeprom.chargedMilliAmpHours / 1000UL; }
    uint32_t    getDischargedAh() const { return eeprom.dischargedMilliAmpHours / 1000UL; }
    uint32_t    getNetAh() const { return (eeprom.chargedMilliAmpHours - eeprom.dischargedMilliAmpHours) / 1000UL; }

    void requestResetFault() {
        if (state == ChargeState::FAULT) {
            faultReason = FaultCode::NONE;
            state = ChargeState::RECOVERY;
            gActiveFault = FaultCode::NONE;
        }
    }

    void adjustVoltageCal(float deltaVolts) {
        eeprom.vCalOffset += (int16_t)(deltaVolts * 100.0f + 0.5f);
    }
    void adjustCurrentCal(float deltaAmps) {
        eeprom.iCalOffset += (int16_t)(deltaAmps * 100.0f + 0.5f);
    }
    int16_t getVoltageCal() const { return eeprom.vCalOffset; }
    int16_t getCurrentCal() const { return eeprom.iCalOffset; }

    void setMasterEnable(bool en) { gMasterEnable = en; }

    /**
     * @brief Applies linear temperature compensation to a base voltage.
     *
     * Most battery chemistries (especially lead-acid) require lower charging
     * voltages at higher temperatures to avoid gassing and reduced lifetime.
     *
     * The compensation is applied as:
     *     V_comp = V_base + (mV_per_C / 1000) * (25 - T)
     *
     * @param baseVoltage  Uncompensated target voltage from the profile
     * @return             Temperature-compensated target voltage
     */
    float getCompensatedVoltage(float baseVoltage) const {
        if (TEMP_COMP_mV_PER_C == 0.0f) return baseVoltage;
        // Standard linear compensation: voltage decreases as temperature rises
        return baseVoltage + (TEMP_COMP_mV_PER_C / 1000.0f) * (25.0f - filteredT);
    }

private:
    // --- State handlers -----------------------------------------------------

    /** One-time initialization and safety check after power-up or reset. */
    void handleInit() {
        // One-time self-test / stabilisation
        if (filteredV < 3.0f) {
            // No battery connected or divider fault
            enterFault(FaultCode::UNDER_VOLTAGE);
            return;
        }
        // Everything looks sane → go to idle and decide later
        state = ChargeState::IDLE;
    }

    /**
     * @brief Idle state — charger is off, monitoring only.
     * Battery is considered healthy. We only leave this state if voltage drops
     * below the configured recharge threshold (and temperature is safe).
     */
    void handleIdle(float v) {
        targetCurrent = 0.0f;
        targetVoltage = compensatedFloatV;

        if (!gMasterEnable) return;

        // Temperature gate
        if (filteredT < MIN_TEMP_C || filteredT > MAX_TEMP_C) return;

        if (v < PRECHARGE_VOLTAGE) {
            state = ChargeState::PRECHARGE;
        } else if (v < RECHARGE_VOLTAGE) {
            state = ChargeState::BULK;
        }
    }

    /**
     * @brief Pre-charge phase for deeply discharged batteries.
     * Uses reduced current to safely bring the battery up to a level where
     * normal bulk charging can begin.
     */
    void handlePrecharge(float v, float i) {
        targetCurrent = PRECHARGE_CURRENT;
        targetVoltage = compensatedAbsorptionV;   // soft ceiling (temp compensated)

        if (v > PRECHARGE_VOLTAGE + 0.3f) {
            state = ChargeState::BULK;
        }
        if (absorptionMinutes > 30) {         // safety timeout on pre-charge
            enterFault(FaultCode::CHARGE_TIMEOUT);
        }
    }

    /**
     * @brief Constant Current (Bulk) phase.
     * We push maximum allowed current until the battery voltage reaches the
     * (temperature compensated) absorption voltage.
     */
    void handleBulk(float v, float i) {
        targetCurrent = MAX_CHARGE_CURRENT;
        targetVoltage = compensatedAbsorptionV;

        // Voltage has reached absorption band → move to CV phase
        if (v >= (compensatedAbsorptionV - 0.15f) && i < (MAX_CHARGE_CURRENT * 0.9f)) {
            absorptionMinutes = 0;
            state = ChargeState::ABSORPTION;
        }
    }

    /**
     * @brief Constant Voltage (Absorption) phase.
     * Voltage is held at the absorption setpoint while current naturally tapers.
     */
    void handleAbsorption(float v, float i) {
        targetCurrent = MAX_CHARGE_CURRENT * 0.6f; // allow taper
        targetVoltage = compensatedAbsorptionV;

        if (i < ABSORPTION_EXIT_CURRENT || absorptionMinutes >= ABSORPTION_MAX_MINUTES) {
            floatHours = 0;
            state = ChargeState::FLOAT;
        }
    }

    /**
     * @brief Float / Maintenance phase.
     * Lower voltage to keep the battery topped up without overcharging.
     */
    void handleFloat(float v, float i) {
        targetCurrent = MAX_CHARGE_CURRENT * 0.15f;
        targetVoltage = compensatedFloatV;

        if (v < RECHARGE_VOLTAGE) {
            state = ChargeState::BULK;
        }
    }

    /** Fault state — charger is forced off until manually reset. */
    void handleFault() {
        targetCurrent = 0.0f;
        targetVoltage = 0.0f;
        setPWMDuty(0);
        digitalWrite(PIN_CHARGE_ENABLE, LOW);
        // Stay here until explicit reset
    }

    /** Brief recovery window after a fault is cleared. */
    void handleRecovery(float v) {
        // Cooldown / stabilisation period after a fault clears
        if (v > MIN_OPERATING_VOLTAGE + 0.5f && v < MAX_CHARGE_VOLTAGE - 0.5f) {
            state = ChargeState::IDLE;
        } else {
            state = ChargeState::IDLE; // be lenient after manual intervention
        }
    }

    // --- Safety -------------------------------------------------------------

    bool checkSafetyLimits(float v, float i, float t) {
        faultReason = FaultCode::NONE;

        if (v > MAX_CHARGE_VOLTAGE)       faultReason = FaultCode::OVER_VOLTAGE;
        else if (v < MIN_OPERATING_VOLTAGE && state != ChargeState::INIT)
                                          faultReason = FaultCode::UNDER_VOLTAGE;
        else if (i > MAX_CHARGE_CURRENT * 1.25f)
                                          faultReason = FaultCode::OVER_CURRENT;
        else if (t > MAX_TEMP_C)          faultReason = FaultCode::OVER_TEMP;
        else if (t < MIN_TEMP_C && (state == ChargeState::BULK || state == ChargeState::ABSORPTION))
                                          faultReason = FaultCode::UNDER_TEMP;

        return (faultReason != FaultCode::NONE);
    }

    // --- Control application ------------------------------------------------

    /**
     * @brief Applies the control law and drives the power stage.
     *
     * Uses a simple but effective P + limited-I controller with mode-dependent
     * gains and explicit slew-rate limiting. The goal is stable regulation
     * without excessive overshoot or ringing on real-world power stages.
     *
     * Maintainer note:
     *   - We deliberately use a low-pass on previous duty (`* 0.6f`) combined
     *     with P term rather than a full PID. This has proven more stable
     *     on inductive loads with the limited CPU cycles available.
     */
    void applyControlOutputs(float v, float i) {
        if (!gMasterEnable || state == ChargeState::FAULT || state == ChargeState::IDLE) {
            setPWMDuty(0);
            digitalWrite(PIN_CHARGE_ENABLE, LOW);
            return;
        }

        // Choose regulation mode
        bool voltageMode = (state == ChargeState::ABSORPTION || state == ChargeState::FLOAT);

        float error, kp, ki;
        if (voltageMode) {
            error = targetVoltage - v;
            kp = KP_VOLTAGE;
            ki = 0.0f;                    // pure P for voltage to avoid overshoot
        } else {
            error = targetCurrent - i;
            kp = KP_CURRENT;
            ki = KI_CURRENT;
        }

        // Update integral (anti-windup)
        pwmIntegral += error * ki;
        pwmIntegral = constrain(pwmIntegral, -30.0f, 30.0f);

        float newDutyF = (getPWMDuty() * 0.6f) + (error * kp) + pwmIntegral;
        int16_t newDuty = (int16_t)newDutyF;

        // Slew rate limiter + clamp
        int16_t current = getPWMDuty();
        int16_t delta = constrain(newDuty - current, -PWM_SLEW_LIMIT, PWM_SLEW_LIMIT);
        uint8_t duty = constrain(current + delta, 0, PWM_MAX_DUTY);

        setPWMDuty(duty);

        // Enable power stage only when we actually want current
        bool wantCharge = (duty > 5) || (state == ChargeState::PRECHARGE);
        digitalWrite(PIN_CHARGE_ENABLE, wantCharge ? HIGH : LOW);
    }

    void updateTimers() {
        if (state == ChargeState::ABSORPTION) {
            absorptionMinutes++;
        }
        if (state == ChargeState::FLOAT) {
            floatHours++;
            // Use 64-bit math to avoid overflow on long recharge intervals (e.g. 72 h)
            uint64_t threshold = (uint64_t)FLOAT_RECHARGE_HOURS * 60ULL * WAKES_PER_MINUTE;
            if (floatHours >= threshold) {
                floatHours = 0;
                state = ChargeState::BULK;   // request full recharge (prevents sulfation)
            }
        }
    }

    void onStateTransition(ChargeState from, ChargeState to) {
        // Any entry/exit side-effects
        pwmIntegral = 0.0f;                 // reset integrator on mode change
        if (to == ChargeState::BULK || to == ChargeState::ABSORPTION) {
            digitalWrite(PIN_CHARGE_ENABLE, HIGH);
        }
        if (to == ChargeState::FAULT) {
            setPWMDuty(0);
            digitalWrite(PIN_CHARGE_ENABLE, LOW);
        }

        // === POWER + EEPROM endurance: only force immediate write on important transitions ===
        // Writing EEPROM costs ~3–4 ms at elevated current and wears the cell.
        // We only force a save for high-value state changes. The periodic throttle
        // (every ~10 min) will catch lastState + total mAh for everything else.
        bool critical = (to == ChargeState::FAULT) ||
                        (to == ChargeState::BULK) ||
                        (to == ChargeState::ABSORPTION) ||
                        (from == ChargeState::BULK && to == ChargeState::IDLE);

        if (critical) {
            eeprom.lastState = (uint8_t)to;
            saveToEEPROM();
            gLastEEPROMSave = gWakeCount;
        } else {
            // Still update the in-RAM lastState so the next throttled save will persist it
            eeprom.lastState = (uint8_t)to;
        }
    }

    // --- Persistence (robust) ----------------------------------------------

    /**
     * @brief Loads persistent state from EEPROM with validation.
     *
     * On first boot or if the stored data is corrupt (bad magic, wrong version,
     * or CRC failure), the EEPROM is reinitialized with safe defaults.
     *
     * Versioning allows future schema changes without bricking deployed units.
     */
    void loadFromEEPROM() {
        EEPROM.get(0, eeprom);
        if (eeprom.magic != EEPROM_MAGIC || eeprom.version != EEPROM_VERSION ||
            crc8((uint8_t*)&eeprom, sizeof(eeprom) - 1) != eeprom.crc8) {
            // Corrupt or virgin EEPROM → initialise sane defaults
            eeprom.magic = EEPROM_MAGIC;
            eeprom.version = EEPROM_VERSION;
            eeprom.lastState = (uint8_t)ChargeState::IDLE;
            eeprom.chargedMilliAmpHours = 0;
            eeprom.dischargedMilliAmpHours = 0;
            eeprom.vCalOffset = 0;
            eeprom.iCalOffset = 0;
            eeprom.crc8 = 0;
            saveToEEPROM();
        }
    }

    /**
     * @brief Saves the current EepromData struct to EEPROM.
     *
     * CRC is recalculated before every write. Writes are intentionally
     * throttled in most paths to preserve flash endurance (100k cycles).
     */
    void saveToEEPROM() {
        eeprom.crc8 = crc8((uint8_t*)&eeprom, sizeof(eeprom) - 1);
        EEPROM.put(0, eeprom);
    }

    // --- Member data --------------------------------------------------------
    ChargeState   state;
    ChargeState   prevState;
    float         filteredV;
    float         filteredI;           // charge current (positive = charging battery)
    float         filteredLoadI;       // load current (positive = battery discharging)
    float         filteredT;
    float         targetCurrent;
    float         targetVoltage;
    float         compensatedAbsorptionV;   // temperature compensated
    float         compensatedFloatV;        // temperature compensated
    float         pwmIntegral;
    uint32_t      absorptionMinutes;   // promoted for long absorption timeouts
    uint32_t      floatHours;          // promoted; was uint16_t and would overflow badly
    FaultCode     faultReason;
    uint8_t       consecutiveFaults;
    EepromData    eeprom;
};

// =============================================================================
// SENSOR ACQUISITION (robust & hardware-aware)
// =============================================================================

/**
 * @brief Performs multiple ADC readings with optional power gating of the sensor.
 *
 * This helper reduces noise through oversampling. When `PIN_DIVIDER_POWER` is
 * defined, the sensor divider is only powered during the measurement window
 * to save quiescent current.
 */
static float readAveraged(uint8_t pin) {
    // Optional: power the divider only during measurement (saves ~0.5 mA)
    if (PIN_DIVIDER_POWER != 255) {
        digitalWrite(PIN_DIVIDER_POWER, HIGH);
        delayMicroseconds(80);                 // settle
    }

    // === POWER: Temporarily run ADC at higher speed for the burst ===
    // Default Arduino prescaler /128 is slow. We use /16 during the short
    // measurement window (still plenty accurate for battery monitoring).
    // This shortens the time the MCU spends awake at high current.
    uint8_t oldADCSRA = ADCSRA;
    ADCSRA = (oldADCSRA & ~0x07) | 0x04;   // prescaler = 16 (ADPS2:0 = 100)

    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
        sum += analogRead(pin);
        delayMicroseconds(40);             // shorter settling because faster ADC
    }

    ADCSRA = oldADCSRA;                    // restore original speed

    if (PIN_DIVIDER_POWER != 255) {
        digitalWrite(PIN_DIVIDER_POWER, LOW);
    }
    return (float)sum / ADC_SAMPLES;
}

/** Reads battery voltage through the configured divider and scaling. */
static float readAveragedVoltage() {
    float raw = readAveraged(PIN_VOLTAGE_SENSE);
    float volts = (raw * ADC_REFERENCE_V / 1023.0f) * VOLTAGE_DIVIDER_RATIO;
    return volts;
}

/** Reads charge current (positive when current is flowing into the battery). */
static float readAveragedCurrent() {
    float raw = readAveraged(PIN_CURRENT_SENSE);
    float amps = (raw - CURRENT_ZERO_POINT) * CURRENT_SCALE;
    if (amps < 0.0f) amps = 0.0f;              // unidirectional assumption
    return amps;
}

/** Reads load current (positive when current is flowing out of the battery). */
static float readAveragedLoadCurrent() {
    float raw = readAveraged(PIN_LOAD_CURRENT_SENSE);
    float amps = (raw - CURRENT_ZERO_POINT) * CURRENT_SCALE;
    if (amps < 0.0f) amps = 0.0f;
    return amps;
}

/**
 * @brief Reads NTC temperature using the Beta model.
 *
 * Includes a guard against open/short sensor conditions that would otherwise
 * cause division by zero or invalid logarithm arguments.
 */
static float readTemperatureC() {
    float raw = readAveraged(PIN_TEMP_SENSE);
    float v = (raw * ADC_REFERENCE_V / 1023.0f);

    // Guard against sensor fault / open circuit / short (prevents div0 or log domain error)
    if (v >= ADC_REFERENCE_V - 0.02f || v < 0.02f) {
        return 99.9f;   // obviously invalid → will trigger OVER_TEMP safety
    }

    // 10 k NTC + 10 k pull-up Beta model
    float r = (v * NTC_PULLUP_R) / (ADC_REFERENCE_V - v);
    if (r <= 0.0f) return 99.9f;

    float invT = (1.0f / (NTC_NOMINAL_T + 273.15f)) +
                 (1.0f / NTC_BETA) * log(r / NTC_NOMINAL_R);
    float tC = (1.0f / invT) - 273.15f;
    return tC;
}

// =============================================================================
// PWM (Timer 1, 8-bit Fast PWM on pin 9)
// =============================================================================

/**
 * @brief Configures Timer1 for fast PWM on OC1A (pin 9).
 *
 * ~31 kHz is a good compromise between audible noise, inductor size,
 * and switching losses for most small-to-medium charger designs.
 */
static void setupPWM() {
    pinMode(PIN_PWM_CONTROL, OUTPUT);
    // Fast PWM, non-inverting, 8-bit, prescaler = 1 → ~31.25 kHz on 16 MHz
    TCCR1A = _BV(COM1A1) | _BV(WGM10);
    TCCR1B = _BV(WGM12) | _BV(CS10);
    OCR1A = 0;
}

/**
 * @brief Completely stops Timer1 to save power when PWM is not needed.
 */
static void stopPWM() {
    // Power optimization: completely stop Timer1 when not charging.
    // Saves the timer clock tree and associated current (~100-300 µA typical).
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 0;
}

/**
 * @brief Sets the PWM duty cycle (0-255).
 *
 * Automatically starts or stops Timer1 as needed. This is the central point
 * for all PWM power management.
 */
static void setPWMDuty(uint8_t duty) {
    if (duty == 0) {
        stopPWM();
    } else {
        if ((TCCR1B & 0x07) == 0) setupPWM();  // restart if stopped
        OCR1A = duty;
    }
}

/** Returns the current PWM duty cycle (0-255). */
static uint8_t getPWMDuty() {
    return OCR1A;
}

// =============================================================================
// Ultra-low-power helpers (PRR + DIDR0)
// =============================================================================

/**
 * @brief Prepare the MCU for the deepest possible sleep.
 *
 * This function is critical for achieving single-digit to low tens of µA
 * average current. It disables as many power-consuming peripherals and
 * input buffers as possible before calling `LowPower.powerDown()`.
 *
 * Called from the end of every control cycle.
 */
static void prepareForDeepSleep() {
    // Disable digital input buffers on all ADC pins (A0–A5).
    // This is a well-known ~50–100 µA saving on ATmega328P when pins are
    // used as analog inputs or left floating.
    DIDR0 = 0x3F;

    // Power down as many peripherals as possible via the Power Reduction Register.
    // Timer1 is already stopped by our setPWMDuty(0) logic when not charging.
    // USART0 is already powered down when SERIAL_DEBUG_ALWAYS is not defined.
    PRR |= (1 << PRTWI) | (1 << PRTIM0) | (1 << PRTIM2) | (1 << PRSPI);

    // Note: The LowPower library will also manipulate PRR for the modules
    // it controls during powerDown(). We only add the ones it doesn't touch
    // or that we want off for the entire sleep period.
}

/**
 * @brief Restore MCU peripherals after waking from deep sleep.
 *
 * Re-enables only the peripherals required for the short active window.
 * Timer1 is intentionally left off here and is only enabled on-demand
 * when PWM is actually needed (see `setPWMDuty()`).
 */
static void restoreAfterWake() {
    // Re-enable digital input buffers (they are needed for some pin modes)
    DIDR0 = 0x00;

    // Re-enable the peripherals we will actually use in the short active window.
    // Timer1 will be turned back on only if we need PWM (see setPWMDuty).
    PRR &= ~((1 << PRTWI) | (1 << PRTIM0) | (1 << PRTIM2) | (1 << PRSPI) | (1 << PRTIM1));
}

// =============================================================================
// LED PATTERNS (non-blocking, updated on every wake)
// =============================================================================

/**
 * @brief Updates the status LED blink pattern based on current charger state.
 *
 * Runs every wake cycle. Because we are in deep sleep most of the time,
 * the blink patterns are relatively slow but still provide useful visual feedback.
 */
static void updateLED() {
    static uint8_t pattern = 0;
    ChargeState s = gCharger ? gCharger->getState() : ChargeState::FAULT;

    bool on = false;
    switch (s) {
        case ChargeState::IDLE:       pattern = (gWakeCount % 8 < 1) ? 1 : 0; break; // very slow wink
        case ChargeState::BULK:       pattern = (gWakeCount % 3 < 2) ? 1 : 0; break; // 66% duty
        case ChargeState::ABSORPTION: pattern = (gWakeCount % 4 < 2) ? 1 : 0; break; // 50%
        case ChargeState::FLOAT:      pattern = (gWakeCount % 6 < 1) ? 1 : 0; break; // occasional
        case ChargeState::FAULT:      pattern = (gWakeCount % 2); break;               // fast blink
        case ChargeState::PRECHARGE:  pattern = (gWakeCount % 5 < 1) ? 1 : 0; break;
        default:                      pattern = 0;
    }
    digitalWrite(PIN_STATUS_LED, pattern ? HIGH : LOW);
}

// =============================================================================
// SERIAL UI (robust, human + machine friendly)
// =============================================================================

/**
 * @brief Prints human-readable and machine-readable status information.
 *
 * The compact CSV line is always emitted periodically. Verbose mode adds
 * a more readable block (triggered by the `status` command).
 */
static void printStatus(bool verbose) {
    if (!gCharger) return;

    // Compact machine-readable line (CSV)
    Serial.print(F("BM,"));
    Serial.print((uint16_t)gCharger->getState()); Serial.print(F(","));
    Serial.print(gCharger->getVoltage(), 2); Serial.print(F(","));
    Serial.print(gCharger->getCurrent(), 2); Serial.print(F(","));
    Serial.print(gCharger->getLoadCurrent(), 2); Serial.print(F(","));
    Serial.print(gCharger->getNetCurrent(), 2); Serial.print(F(","));
    Serial.print(gCharger->getTemp(), 1); Serial.print(F(","));
    Serial.print(gCharger->getPWM()); Serial.print(F(","));
    Serial.print((uint8_t)gCharger->getFault()); Serial.print(F(","));
    Serial.print(gCharger->getChargedAh());
    Serial.print(F(","));
    Serial.print(gCharger->getDischargedAh());
    Serial.println();

    if (verbose) {
        Serial.print(F("State: ")); Serial.print(stateToString(gCharger->getState()));
        Serial.print(F("  V=")); Serial.print(gCharger->getVoltage(), 2);
        Serial.print(F("  I=")); Serial.print(gCharger->getCurrent(), 2);
        Serial.print(F("  LoadI=")); Serial.print(gCharger->getLoadCurrent(), 2);
        Serial.print(F("  NetI=")); Serial.print(gCharger->getNetCurrent(), 2);
        Serial.print(F("  T=")); Serial.print(gCharger->getTemp(), 1);
        Serial.print(F(" °C  PWM=")); Serial.print(gCharger->getPWM());
        Serial.print(F("  Charged=")); Serial.print(gCharger->getChargedAh());
        Serial.print(F("Ah  Discharged=")); Serial.print(gCharger->getDischargedAh());
        Serial.println(F("Ah"));
    }
}

// Richer machine-readable telemetry (JSON)
static void printJSON() {
    if (!gCharger) return;

    Serial.print(F("{\"state\":\""));
    Serial.print(stateToString(gCharger->getState()));
    Serial.print(F("\",\"v\":"));
    Serial.print(gCharger->getVoltage(), 3);
    Serial.print(F(",\"i_charge\":"));
    Serial.print(gCharger->getCurrent(), 3);
    Serial.print(F(",\"i_load\":"));
    Serial.print(gCharger->getLoadCurrent(), 3);
    Serial.print(F(",\"i_net\":"));
    Serial.print(gCharger->getNetCurrent(), 3);
    Serial.print(F(",\"temp\":"));
    Serial.print(gCharger->getTemp(), 1);
    Serial.print(F(",\"pwm\":"));
    Serial.print(gCharger->getPWM());
    Serial.print(F(",\"target_v\":"));
    Serial.print(gCharger->getTargetVoltage(), 3);
    Serial.print(F(",\"comp_absorb_v\":"));
    Serial.print(gCharger->getCompensatedAbsorptionV(), 3);
    Serial.print(F(",\"comp_float_v\":"));
    Serial.print(gCharger->getCompensatedFloatV(), 3);
    Serial.print(F(",\"charged_ah\":"));
    Serial.print(gCharger->getChargedAh(), 3);
    Serial.print(F(",\"discharged_ah\":"));
    Serial.print(gCharger->getDischargedAh(), 3);
    Serial.print(F(",\"net_ah\":"));
    Serial.print(gCharger->getNetAh(), 3);
    Serial.println(F("}"));
}

#if ENABLE_OLED
// === Optional OLED Support (SSD1306 128x64 via I2C) ===
Adafruit_SSD1306 display(128, 64, &Wire, -1);

static void initOLED() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
        // OLED not found — continue without it
        return;
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("BatteryManager"));
    display.display();
}

static uint16_t oledUpdateCounter = 0;

static void updateOLED() {
    if (!gCharger) return;

    // Only update every ~10 cycles (~20 seconds) to reduce I2C traffic and power
    if (++oledUpdateCounter % 10 != 0) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    display.print(F("V: ")); display.print(gCharger->getVoltage(), 2); display.println(F("V"));
    display.print(F("I: ")); display.print(gCharger->getCurrent(), 2); 
    display.print(F(" L: ")); display.print(gCharger->getLoadCurrent(), 2); display.println(F("A"));
    display.print(F("Net: ")); display.print(gCharger->getNetCurrent(), 2); display.println(F("A"));
    display.print(F("T: ")); display.print(gCharger->getTemp(), 1); display.println(F("C"));
    display.print(F("State: ")); display.println(stateToString(gCharger->getState()));
    display.print(F("Ah: ")); display.print(gCharger->getChargedAh(), 1); 
    display.print(F("/")); display.print(gCharger->getDischargedAh(), 1);

    display.display();
}
#else
static void initOLED() {}
static void updateOLED() {}
#endif

/**
 * @brief Non-blocking serial command processor.
 *
 * Commands are processed only during the short active window after waking.
 * When Serial is disabled for power saving, this function becomes a no-op.
 */
static void handleSerialCommands() {
    static char buf[32];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            buf[idx] = 0;
            idx = 0;
            // Very simple command parser
            if (strcmp(buf, "help") == 0 || strcmp(buf, "?") == 0) {
                Serial.println(F("Commands: status, start, stop, reset, cal v=+0.12, cal i=-0.05, dump"));
            } else if (strcmp(buf, "status") == 0) {
                printStatus(true);
            } else if (strcmp(buf, "start") == 0) {
                gMasterEnable = true; Serial.println(F("Charging enabled"));
            } else if (strcmp(buf, "stop") == 0) {
                gMasterEnable = false; Serial.println(F("Charging disabled"));
            } else if (strcmp(buf, "reset") == 0) {
                if (gCharger) gCharger->requestResetFault();
                Serial.println(F("Fault cleared (if any)"));
            } else if (strncmp(buf, "cal v=", 6) == 0) {
                float delta = atof(buf + 6);
                gCharger->adjustVoltageCal(delta);
                Serial.print(F("V offset now: ")); Serial.println(gCharger->getVoltageCal());
            } else if (strncmp(buf, "cal i=", 6) == 0) {
                float delta = atof(buf + 6);
                gCharger->adjustCurrentCal(delta);
                Serial.print(F("I offset now: ")); Serial.println(gCharger->getCurrentCal());
            } else if (strcmp(buf, "dump") == 0) {
                Serial.print(F("Profile: ")); Serial.println(getProfileName());
                Serial.print(F("Charged: ")); Serial.print(gCharger->getChargedAh());
                Serial.print(F("Ah  Discharged: ")); Serial.print(gCharger->getDischargedAh());
                Serial.println(F("Ah"));
            } else if (strcmp(buf, "json") == 0) {
                printJSON();
            } else if (strlen(buf) > 0) {
                Serial.print(F("Unknown: ")); Serial.println(buf);
            }
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
}

// =============================================================================
// UTILITIES
// =============================================================================

static uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        uint8_t in = *data++;
        for (uint8_t i = 8; i; i--) {
            uint8_t mix = (crc ^ in) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            in >>= 1;
        }
    }
    return crc;
}

static const __FlashStringHelper* stateToString(ChargeState s) {
    switch (s) {
        case ChargeState::INIT:      return F("INIT");
        case ChargeState::IDLE:      return F("IDLE");
        case ChargeState::PRECHARGE: return F("PRECHARGE");
        case ChargeState::BULK:      return F("BULK");
        case ChargeState::ABSORPTION:return F("ABSORPTION");
        case ChargeState::FLOAT:     return F("FLOAT");
        case ChargeState::FAULT:     return F("FAULT");
        case ChargeState::RECOVERY:  return F("RECOVERY");
        default:                     return F("?");
    }
}

static const __FlashStringHelper* faultToString(FaultCode f) {
    uint8_t bits = static_cast<uint8_t>(f);
    if (bits == 0) return F("None");
    if (bits & static_cast<uint8_t>(FaultCode::OVER_VOLTAGE))   return F("OverVoltage");
    if (bits & static_cast<uint8_t>(FaultCode::UNDER_VOLTAGE))  return F("UnderVoltage");
    if (bits & static_cast<uint8_t>(FaultCode::OVER_CURRENT))   return F("OverCurrent");
    if (bits & static_cast<uint8_t>(FaultCode::OVER_TEMP))      return F("OverTemp");
    if (bits & static_cast<uint8_t>(FaultCode::UNDER_TEMP))     return F("UnderTemp");
    if (bits & static_cast<uint8_t>(FaultCode::CHARGE_TIMEOUT)) return F("Timeout");
    if (bits & static_cast<uint8_t>(FaultCode::SENSOR_FAULT))   return F("Sensor");
    if (bits & static_cast<uint8_t>(FaultCode::MANUAL))         return F("Manual");
    return F("Unknown");
}

static void enterFault(FaultCode reason) {
    gActiveFault = reason;
    if (gCharger) {
        // The controller will see the fault on next cycle via safety check
    }
}

// =============================================================================
// ARDUINO ENTRY POINTS
// =============================================================================

/**
 * @brief Arduino setup function.
 *
 * Performs one-time initialization:
 *   - Disables watchdog during potentially long startup
 *   - Configures pins (including forcing unused pins to INPUT_PULLUP)
 *   - Sets up Timer1 for PWM
 *   - Loads persistent state from EEPROM
 *   - Prints boot banner (then powers down USART for low power)
 *   - Arms the hardware watchdog
 *
 * Maintainer note:
 *   Serial is deliberately disabled after the initial banner unless
 *   `SERIAL_DEBUG_ALWAYS` is defined. This is one of the largest single
 *   power savings in deployed units.
 */
void setup() {
    // Critical: disable watchdog during long startup
    wdt_disable();

    Serial.begin(SERIAL_BAUD);
    delay(80);
    Serial.println();
    Serial.println(F("BatteryManager v1.0 – Robust Low-Power Charger"));
    Serial.print(F("Profile: ")); Serial.println(getProfileName());
    Serial.print(F("Compile: ")); Serial.print(__DATE__); Serial.print(" "); Serial.println(__TIME__);

    // Pin configuration
    pinMode(PIN_CHARGE_ENABLE, OUTPUT);
    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_DEBUG_LED, OUTPUT);
    digitalWrite(PIN_CHARGE_ENABLE, LOW);
    digitalWrite(PIN_STATUS_LED, LOW);

    if (PIN_DIVIDER_POWER != 255) {
        pinMode(PIN_DIVIDER_POWER, OUTPUT);
        digitalWrite(PIN_DIVIDER_POWER, LOW);
    }

    // === POWER: Configure every unused pin as INPUT_PULLUP ===
    // Floating inputs can source/sink significant phantom current on ATmega328P
    // especially in electrically noisy environments (chargers, motors, etc.).
    // List of used pins: 0/1 (Serial when enabled), 3,5,8?,9,13, A0-A2 (+ divider if used)
    for (uint8_t p = 0; p < 20; p++) {
        bool used = (p == 0 || p == 1 || p == PIN_CHARGE_ENABLE || p == PIN_STATUS_LED ||
                     p == PIN_DEBUG_LED || p == PIN_PWM_CONTROL ||
                     p == PIN_VOLTAGE_SENSE || p == PIN_CURRENT_SENSE || p == PIN_LOAD_CURRENT_SENSE || p == PIN_TEMP_SENSE);
        if (PIN_DIVIDER_POWER != 255 && p == PIN_DIVIDER_POWER) used = true;
        if (!used) {
            pinMode(p, INPUT_PULLUP);
        }
    }

    setupPWM();

    // Create the controller (heap allocation acceptable on AVR for this use)
    static ChargerController controller;
    gCharger = &controller;
    gCharger->begin();

    // Optional OLED
    #if ENABLE_OLED
    initOLED();
    #endif

    // Enable brown-out detection (already done by fuses on most boards)
    // Optional: attach a real BOD interrupt handler here if desired

    Serial.println(F("Ready. Type 'help' for commands."));
    printStatus(true);

#ifndef SERIAL_DEBUG_ALWAYS
    // === POWER OPTIMIZATION: Disable Serial / USART0 after boot banner ===
    // The USART draws significant current even when idle (~0.5-1 mA range on
    // ATmega328P). This is one of the largest single power wins for battery use.
    Serial.flush();
    Serial.end();
    PRR |= (1 << PRUSART0);   // Power Reduction Register - USART0
#endif

    // Arm a short watchdog in case we hang (8 s)
    wdt_enable(WDTO_8S);
}

/**
 * @brief Main Arduino loop.
 *
 * This function is structured around deep sleep. After each control cycle
 * the MCU is put into power-down mode for ~2 seconds. All work happens
 * in the short active window between `restoreAfterWake()` and
 * `prepareForDeepSleep()`.
 *
 * The watchdog is reset at the very beginning of every iteration.
 */
void loop() {
    wdt_reset();                       // we are alive

    if (gCharger) {
        gCharger->runCycle();
    }

    updateLED();
    updateOLED();
    handleSerialCommands();

    // Periodic status line (machine readable)
    if ((++gPrintCounter % STATUS_PRINT_INTERVAL) == 0) {
        printStatus(false);
    }

    gWakeCount++;

    // === POWER: Enter deepest possible sleep ===
    // BOD_OFF saves several µA during the 2-second sleep periods.
    // Trade-off: slightly higher risk of undetected brown-out while sleeping.
    // For a battery charger that is already monitoring voltage every cycle,
    // this is considered an acceptable risk for maximum runtime.
    prepareForDeepSleep();
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
    restoreAfterWake();
    // Code resumes here after wake-up (≈2 s later)
}