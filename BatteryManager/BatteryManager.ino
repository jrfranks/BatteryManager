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
 *          Algorithmic core lives in src/core/ (header-only). This sketch owns
 *          the HAL, Arduino setup()/loop(), and AVR peripherals.
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
#include "src/core/charger_controller.h"
#include "src/core/led_pattern.h"
#include "src/core/sensor_math.h"
#include "src/core/serial_parse.h"
#include "src/core/telemetry.h"

#include <EEPROM.h>
#include <LowPower.h>
#include <avr/wdt.h>

#if ENABLE_OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
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
// FORWARD DECLARATIONS
// =============================================================================

static void setupPWM();
static void setPWMDuty(uint8_t duty);
static uint8_t getPWMDuty();
static float readAveragedVoltage();
static float readAveragedCurrent();
static float readAveragedLoadCurrent();
static float readTemperatureC();
static void printStatus(bool verbose = false);
static void handleSerialCommands();
static void updateLED();
static const __FlashStringHelper *stateToString(ChargeState s);
static const __FlashStringHelper *faultToString(FaultCode f);

// =============================================================================
// GLOBAL STATE (minimal, clearly owned)
// =============================================================================

ChargerController *gCharger = nullptr;

uint32_t gWakeCount = 0;
uint32_t gLastEEPROMSave = 0;
uint32_t gPrintCounter = 0;
bool gMasterEnable = true;

// =============================================================================
// HAL (AVR implementations — controller never touches registers directly)
// =============================================================================

static void halSetChargeEnable(bool on) { digitalWrite(PIN_CHARGE_ENABLE, on ? HIGH : LOW); }

static void halEepromLoad(EepromData *data) { EEPROM.get(0, *data); }

static void halEepromSave(const EepromData *data) { EEPROM.put(0, *data); }

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
static float readAveraged(uint8_t pin)
{
    // Optional: power the divider only during measurement (saves ~0.5 mA)
    if (PIN_DIVIDER_POWER != 255) {
        digitalWrite(PIN_DIVIDER_POWER, HIGH);
        delayMicroseconds(80); // settle
    }

    // === POWER: Temporarily run ADC at higher speed for the burst ===
    // Default Arduino prescaler /128 is slow. We use /16 during the short
    // measurement window (still plenty accurate for battery monitoring).
    // This shortens the time the MCU spends awake at high current.
    uint8_t oldADCSRA = ADCSRA;
    ADCSRA = (oldADCSRA & ~0x07) | 0x04; // prescaler = 16 (ADPS2:0 = 100)

    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
        sum += analogRead(pin);
        delayMicroseconds(40); // shorter settling because faster ADC
    }

    ADCSRA = oldADCSRA; // restore original speed

    if (PIN_DIVIDER_POWER != 255) {
        digitalWrite(PIN_DIVIDER_POWER, LOW);
    }
    return (float)sum / ADC_SAMPLES;
}

static float readAveragedVoltage() { return adcToVoltage(readAveraged(PIN_VOLTAGE_SENSE)); }

static float readAveragedCurrent() { return adcToCurrent(readAveraged(PIN_CURRENT_SENSE)); }

static float readAveragedLoadCurrent() { return adcToCurrent(readAveraged(PIN_LOAD_CURRENT_SENSE)); }

static float readTemperatureC() { return ntcRawToTempC(readAveraged(PIN_TEMP_SENSE)); }

// =============================================================================
// PWM (Timer 1, 8-bit Fast PWM on pin 9)
// =============================================================================

/**
 * @brief Configures Timer1 for fast PWM on OC1A (pin 9).
 *
 * ~31 kHz is a good compromise between audible noise, inductor size,
 * and switching losses for most small-to-medium charger designs.
 */
static void setupPWM()
{
    pinMode(PIN_PWM_CONTROL, OUTPUT);
    // Fast PWM, non-inverting, 8-bit, prescaler = 1 → ~31.25 kHz on 16 MHz
    TCCR1A = _BV(COM1A1) | _BV(WGM10);
    TCCR1B = _BV(WGM12) | _BV(CS10);
    OCR1A = 0;
}

static void stopPWM()
{
    // Power optimization: completely stop Timer1 when not charging.
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 0;
}

static void setPWMDuty(uint8_t duty)
{
    if (duty == 0) {
        stopPWM();
    } else {
        if ((TCCR1B & 0x07) == 0)
            setupPWM(); // restart if stopped
        OCR1A = duty;
    }
}

static uint8_t getPWMDuty() { return OCR1A; }

// =============================================================================
// Ultra-low-power helpers (PRR + DIDR0)
// =============================================================================

static void prepareForDeepSleep()
{
    DIDR0 = 0x3F;
    PRR |= (1 << PRTWI) | (1 << PRTIM0) | (1 << PRTIM2) | (1 << PRSPI);
}

static void restoreAfterWake()
{
    DIDR0 = 0x00;
    PRR &= ~((1 << PRTWI) | (1 << PRTIM0) | (1 << PRTIM2) | (1 << PRSPI) | (1 << PRTIM1));
}

// =============================================================================
// LED PATTERNS (non-blocking, updated on every wake)
// =============================================================================

static void updateLED()
{
    ChargeState s = gCharger ? gCharger->getState() : ChargeState::FAULT;
    bool on = ledOnForState(s, gWakeCount);
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

// =============================================================================
// SERIAL UI (robust, human + machine friendly)
// =============================================================================

static void printStatus(bool verbose)
{
    if (!gCharger)
        return;

    char csv[160];
    formatTelemetryCsv(csv, sizeof(csv), *gCharger);
    Serial.println(csv);

    if (verbose) {
        Serial.print(F("State: "));
        Serial.print(stateToString(gCharger->getState()));
        Serial.print(F("  Fault: "));
        Serial.print(faultToString(gCharger->getFault()));
        Serial.print(F("  V="));
        Serial.print(gCharger->getVoltage(), 2);
        Serial.print(F("  I="));
        Serial.print(gCharger->getCurrent(), 2);
        Serial.print(F("  LoadI="));
        Serial.print(gCharger->getLoadCurrent(), 2);
        Serial.print(F("  NetI="));
        Serial.print(gCharger->getNetCurrent(), 2);
        Serial.print(F("  T="));
        Serial.print(gCharger->getTemp(), 1);
        Serial.print(F(" °C  PWM="));
        Serial.print(gCharger->getPWM());
        Serial.print(F("  Charged="));
        Serial.print(gCharger->getChargedAh());
        Serial.print(F("Ah  Discharged="));
        Serial.print(gCharger->getDischargedAh());
        Serial.println(F("Ah"));
    }
}

static void printJSON()
{
    if (!gCharger)
        return;
    char json[320];
    formatTelemetryJson(json, sizeof(json), *gCharger);
    Serial.println(json);
}

#if ENABLE_OLED
Adafruit_SSD1306 display(128, 64, &Wire, -1);

static void initOLED()
{
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
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

static void updateOLED()
{
    if (!gCharger)
        return;

    if (++oledUpdateCounter % 10 != 0)
        return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    display.print(F("V: "));
    display.print(gCharger->getVoltage(), 2);
    display.println(F("V"));
    display.print(F("I: "));
    display.print(gCharger->getCurrent(), 2);
    display.print(F(" L: "));
    display.print(gCharger->getLoadCurrent(), 2);
    display.println(F("A"));
    display.print(F("Net: "));
    display.print(gCharger->getNetCurrent(), 2);
    display.println(F("A"));
    display.print(F("T: "));
    display.print(gCharger->getTemp(), 1);
    display.println(F("C"));
    display.print(F("State: "));
    display.println(stateToString(gCharger->getState()));
    display.print(F("Ah: "));
    display.print(gCharger->getChargedAh(), 1);
    display.print(F("/"));
    display.print(gCharger->getDischargedAh(), 1);

    display.display();
}
#else
static void initOLED() {}
static void updateOLED() {}
#endif

static void handleSerialCommands()
{
    if (!gCharger) {
        return;
    }
    static SerialCmdBuffer line;

    while (Serial.available()) {
        char c = Serial.read();
        ParsedCommand parsed{};
        if (line.feed(c, parsed) != SerialCmdBuffer::FeedResult::Line) {
            continue;
        }

        switch (parsed.cmd) {
        case Command::Help:
            Serial.println(F("Commands: status, start, stop, reset, cal v=+0.12, cal i=-0.05, dump, json"));
            break;
        case Command::Status:
            printStatus(true);
            break;
        case Command::Start:
            executeCommand(parsed, *gCharger);
            Serial.println(F("Charging enabled"));
            break;
        case Command::Stop:
            executeCommand(parsed, *gCharger);
            Serial.println(F("Charging disabled"));
            break;
        case Command::Reset:
            executeCommand(parsed, *gCharger);
            Serial.println(F("Fault cleared (if any)"));
            break;
        case Command::CalV:
            executeCommand(parsed, *gCharger);
            Serial.print(F("V offset now: "));
            Serial.println(gCharger->getVoltageCal());
            break;
        case Command::CalI:
            executeCommand(parsed, *gCharger);
            Serial.print(F("I offset now: "));
            Serial.println(gCharger->getCurrentCal());
            break;
        case Command::Dump:
            Serial.print(F("Profile: "));
            Serial.println(getProfileName());
            Serial.print(F("Charged: "));
            Serial.print(gCharger->getChargedAh());
            Serial.print(F("Ah  Discharged: "));
            Serial.print(gCharger->getDischargedAh());
            Serial.println(F("Ah"));
            break;
        case Command::Json:
            printJSON();
            break;
        case Command::Unknown:
            Serial.print(F("Unknown: "));
            Serial.println(line.buf);
            break;
        case Command::None:
        default:
            break;
        }
    }
}

// =============================================================================
// UTILITIES
// =============================================================================

static const __FlashStringHelper *stateToString(ChargeState s)
{
    return reinterpret_cast<const __FlashStringHelper *>(chargeStateName(s));
}

static const __FlashStringHelper *faultToString(FaultCode f)
{
    return reinterpret_cast<const __FlashStringHelper *>(faultName(f));
}

// =============================================================================
// ARDUINO ENTRY POINTS
// =============================================================================

void setup()
{
    wdt_disable();

    Serial.begin(SERIAL_BAUD);
    delay(80);
    Serial.println();
    Serial.println(F("BatteryManager v1.0 – Robust Low-Power Charger"));
    Serial.print(F("Profile: "));
    Serial.println(getProfileName());
    Serial.print(F("Compile: "));
    Serial.print(__DATE__);
    Serial.print(" ");
    Serial.println(__TIME__);

    pinMode(PIN_CHARGE_ENABLE, OUTPUT);
    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_DEBUG_LED, OUTPUT);
    digitalWrite(PIN_CHARGE_ENABLE, LOW);
    digitalWrite(PIN_STATUS_LED, LOW);

    if (PIN_DIVIDER_POWER != 255) {
        pinMode(PIN_DIVIDER_POWER, OUTPUT);
        digitalWrite(PIN_DIVIDER_POWER, LOW);
    }

    for (uint8_t p = 0; p < 20; p++) {
        bool used = (p == 0 || p == 1 || p == PIN_CHARGE_ENABLE || p == PIN_STATUS_LED || p == PIN_DEBUG_LED ||
                     p == PIN_PWM_CONTROL || p == PIN_VOLTAGE_SENSE || p == PIN_CURRENT_SENSE ||
                     p == PIN_LOAD_CURRENT_SENSE || p == PIN_TEMP_SENSE);
        if (PIN_DIVIDER_POWER != 255 && p == PIN_DIVIDER_POWER)
            used = true;
        if (!used) {
            pinMode(p, INPUT_PULLUP);
        }
    }

    setupPWM();

    static ChargerHal hal{};
    hal.readVoltage = readAveragedVoltage;
    hal.readCurrent = readAveragedCurrent;
    hal.readLoadCurrent = readAveragedLoadCurrent;
    hal.readTempC = readTemperatureC;
    hal.getPwm = getPWMDuty;
    hal.setPwm = setPWMDuty;
    hal.setChargeEnable = halSetChargeEnable;
    hal.eepromLoad = halEepromLoad;
    hal.eepromSave = halEepromSave;
    hal.wakeCount = &gWakeCount;
    hal.lastEepromSave = &gLastEEPROMSave;
    hal.masterEnable = &gMasterEnable;

    static ChargerController controller(hal);
    gCharger = &controller;
    gCharger->begin();

    initOLED();

    Serial.println(F("Ready. Type 'help' for commands."));
    printStatus(true);

#ifndef SERIAL_DEBUG_ALWAYS
    Serial.flush();
    Serial.end();
    PRR |= (1 << PRUSART0);
#endif

    wdt_enable(WDTO_8S);
}

void loop()
{
    wdt_reset();

    if (gCharger) {
        gCharger->runCycle();
    }

    updateLED();
    updateOLED();
    handleSerialCommands();

    if ((++gPrintCounter % STATUS_PRINT_INTERVAL) == 0) {
        printStatus(false);
    }

    gWakeCount++;

    prepareForDeepSleep();
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
    restoreAfterWake();
}
