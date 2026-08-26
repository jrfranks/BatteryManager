/**
 * @file    Config.h
 * @brief   Central configuration and tuning surface for BatteryManager.
 *
 * @details This is the **primary file** a maintainer or end-user will edit.
 *          It contains every tunable parameter in one place:
 *            - Hardware pin mapping
 *            - Battery chemistry profiles (with temperature coefficients)
 *            - Sensor calibration and scaling
 *            - Safety limits, timing, and control gains
 *            - Feature flags (OLED, debug serial, etc.)
 *            - EEPROM data layout and versioning
 *
 * @warning  **Edit this file with extreme care.** Incorrect calibration
 *           (especially VOLTAGE_DIVIDER_RATIO and CURRENT_SCALE) can cause
 *           overcharging, fire, or battery damage. Always verify values with
 *           a calibrated multimeter.
 *
 * @section How to Use
 *   1. Uncomment **exactly one** `BATTERY_PROFILE_*` define.
 *   2. Measure your hardware and update the calibration constants below.
 *   3. Adjust safety limits and timing if your setup differs significantly.
 *   4. Enable optional features (`ENABLE_OLED`, etc.) as needed.
 *
 * @section Adding a New Profile
 *   To add a new battery chemistry:
 *     1. Add a new `#elif defined(BATTERY_PROFILE_XXX)` block.
 *     2. Define all required constants (voltages, currents, temps, timers).
 *     3. Update the profile selection comment at the top of this section.
 *
 * @version 1.1
 * @date    2026
 * @author  BatteryManager contributors
 * @license MIT
 *
 * @see DESIGN.md for the reasoning behind power strategy, control law,
 *      temperature compensation, and coulomb counting design.
 */

#pragma once

#ifndef BATTERY_MANAGER_CONFIG_H
#define BATTERY_MANAGER_CONFIG_H

#include <stdint.h>

#ifndef A0
#define A0 14
#endif
#ifndef A1
#define A1 15
#endif
#ifndef A2
#define A2 16
#endif
#ifndef A3
#define A3 17
#endif
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// =============================================================================
// HARDWARE PIN MAPPING (Arduino Uno / Nano / Pro Mini compatible)
// =============================================================================
// All pins are defined as constexpr so they can be used in compile-time checks
// and give good compiler error messages when misused.
//
// Notes for maintainers:
//   - PIN_DIVIDER_POWER is used to power-gate the voltage divider during sleep.
//   - PIN_LOAD_CURRENT_SENSE enables true coulomb counting (charge vs discharge).
//   - When adding new hardware, also update the pin configuration loop in setup().

// === Core Sensor Inputs ===
constexpr uint8_t PIN_VOLTAGE_SENSE = A0;      ///< Battery voltage (through divider)
constexpr uint8_t PIN_CURRENT_SENSE = A1;      ///< Charge current (positive = charging battery)
constexpr uint8_t PIN_LOAD_CURRENT_SENSE = A3; ///< Load current (positive = battery discharging)
constexpr uint8_t PIN_TEMP_SENSE = A2;         ///< Temperature sensor (10k NTC recommended)

// === Power Stage Control ===
constexpr uint8_t PIN_CHARGE_ENABLE = 3; ///< Active HIGH — enables the charger power stage / buck converter
constexpr uint8_t PIN_PWM_CONTROL = 9;   ///< Timer1 PWM output for current/voltage setpoint

// === User Feedback ===
constexpr uint8_t PIN_STATUS_LED = 5;          ///< Main status LED (patterns indicate state)
constexpr uint8_t PIN_DEBUG_LED = LED_BUILTIN; ///< Secondary LED (usually pin 13)

// === Optional / Future Pins ===
// Set to 255 in code to disable the feature
constexpr uint8_t PIN_BUZZER = 6;
constexpr uint8_t PIN_FAN_CONTROL = 7;
constexpr uint8_t PIN_DIVIDER_POWER = 8; ///< Drive HIGH only during ADC reads to save quiescent current

// === Feature Flags ===
// These control compile-time inclusion of optional functionality.
#define ENABLE_OLED 0         ///< Set to 1 to enable SSD1306 OLED (requires Adafruit libraries)
#define OLED_I2C_ADDRESS 0x3C ///< I2C address for most 128x64 SSD1306 modules

// =============================================================================
// BATTERY CHEMISTRY PROFILES
// =============================================================================
// Each profile defines safe operating limits, charging voltages, currents,
// temperature compensation, and timeout values.
//
// To add support for a new chemistry:
//   1. Add a new `#elif defined(BATTERY_PROFILE_XXX)` block below.
//   2. Define all required constants.
//   3. Choose appropriate temperature compensation coefficient (mV/°C).
//
// Only ONE profile may be active at compile time.

/// Select exactly one profile (or pass -DBATTERY_PROFILE_* from the build system):
// #define BATTERY_PROFILE_LEAD_ACID_12V
// #define BATTERY_PROFILE_LIFEPO4_4S
// #define BATTERY_PROFILE_LIION_4S
// #define BATTERY_PROFILE_CUSTOM

#if !defined(BATTERY_PROFILE_LEAD_ACID_12V) && !defined(BATTERY_PROFILE_LIFEPO4_4S) &&                                 \
  !defined(BATTERY_PROFILE_LIION_4S) && !defined(BATTERY_PROFILE_CUSTOM)
#define BATTERY_PROFILE_LEAD_ACID_12V
#endif

// -----------------------------------------------------------------------------
// Active Profile Parameters
// -----------------------------------------------------------------------------
// These values are selected based on the #define above.
#if defined(BATTERY_PROFILE_LEAD_ACID_12V)
constexpr const char *PROFILE_NAME = "Lead-Acid 12V (SLA/GEL/AGM)";
constexpr float NOMINAL_VOLTAGE = 12.0f;       // V
constexpr float ABSORPTION_VOLTAGE = 14.40f;   // V  (temperature compensated)
constexpr float FLOAT_VOLTAGE = 13.50f;        // V
constexpr float PRECHARGE_VOLTAGE = 11.50f;    // V  below this → gentle pre-charge
constexpr float RECHARGE_VOLTAGE = 12.60f;     // V  below this (after float) → restart bulk
constexpr float MAX_CHARGE_VOLTAGE = 15.00f;   // V  hard safety limit
constexpr float MIN_OPERATING_VOLTAGE = 9.50f; // V  below this = deep discharge or bad connection

constexpr float MAX_CHARGE_CURRENT = 5.0f;      // A  (set by your power stage / wiring)
constexpr float PRECHARGE_CURRENT = 0.8f;       // A
constexpr float ABSORPTION_EXIT_CURRENT = 0.4f; // A  or C/20–C/30 typical

constexpr float MAX_TEMP_C = 50.0f;  // °C  stop charging above
constexpr float MIN_TEMP_C = -10.0f; // °C  do not charge frozen batteries

/// Temperature compensation coefficient (negative: hotter → lower charge voltage).
/// Formula used: V_comp = V_base + (TEMP_COMP_mV_PER_C / 1000.0f) * (T - 25.0f)
/// Lead-acid is very sensitive to temperature — always use a negative value.
constexpr float TEMP_COMP_mV_PER_C = -25.0f; // mV/°C for lead-acid (typical range: -20 to -30)

constexpr uint16_t ABSORPTION_MAX_MINUTES = 240; // 4 hours max absorption
constexpr uint16_t FLOAT_RECHARGE_HOURS = 72;    // force bulk check every 3 days

#elif defined(BATTERY_PROFILE_LIFEPO4_4S)
constexpr const char *PROFILE_NAME = "LiFePO4 4S (12.8V nominal)";
constexpr float NOMINAL_VOLTAGE = 12.8f;
constexpr float ABSORPTION_VOLTAGE = 14.40f; // 3.6 V/cell
constexpr float FLOAT_VOLTAGE = 13.60f;      // 3.4 V/cell typical
constexpr float PRECHARGE_VOLTAGE = 12.00f;
constexpr float RECHARGE_VOLTAGE = 13.20f;
constexpr float MAX_CHARGE_VOLTAGE = 14.80f;
constexpr float MIN_OPERATING_VOLTAGE = 10.00f;

constexpr float MAX_CHARGE_CURRENT = 5.0f;
constexpr float PRECHARGE_CURRENT = 1.0f;
constexpr float ABSORPTION_EXIT_CURRENT = 0.25f;

constexpr float MAX_TEMP_C = 55.0f;
constexpr float MIN_TEMP_C = 0.0f; // LiFePO4 tolerates colder but conservative

/// LiFePO4 has milder temperature sensitivity than lead-acid.
constexpr float TEMP_COMP_mV_PER_C = -10.0f; // milder compensation (typical: -5 to -15)

constexpr uint16_t ABSORPTION_MAX_MINUTES = 120;
constexpr uint16_t FLOAT_RECHARGE_HOURS = 48;

#elif defined(BATTERY_PROFILE_LIION_4S)
constexpr const char *PROFILE_NAME = "Li-ion 4S (14.4-14.8V)";
constexpr float NOMINAL_VOLTAGE = 14.4f;
constexpr float ABSORPTION_VOLTAGE = 16.80f; // 4.20 V/cell – use with great caution!
constexpr float FLOAT_VOLTAGE = 16.00f;      // many BMS prefer no float or very low
constexpr float PRECHARGE_VOLTAGE = 13.00f;
constexpr float RECHARGE_VOLTAGE = 15.20f;
constexpr float MAX_CHARGE_VOLTAGE = 17.00f;
constexpr float MIN_OPERATING_VOLTAGE = 11.00f;

constexpr float MAX_CHARGE_CURRENT = 3.0f; // lower for safety on software charger
constexpr float PRECHARGE_CURRENT = 0.5f;
constexpr float ABSORPTION_EXIT_CURRENT = 0.15f;

constexpr float MAX_TEMP_C = 45.0f;
constexpr float MIN_TEMP_C = 5.0f;

/// Li-ion compensation — use with caution. Many modern packs have built-in BMS.
constexpr float TEMP_COMP_mV_PER_C = -15.0f; // typical range for Li-ion: -10 to -20

constexpr uint16_t ABSORPTION_MAX_MINUTES = 90;
constexpr uint16_t FLOAT_RECHARGE_HOURS = 24;

#else
#error "No battery profile defined – please select one in Config.h"
#endif

// =============================================================================
// SENSOR CALIBRATION (hardware dependent – MEASURE AND TUNE THESE)
// =============================================================================
// These values must be measured on your actual hardware. Never rely on the
// default numbers for production use.
//
// Recommended calibration procedure:
//   1. Voltage: Apply known stable voltages (e.g. 12.00V and 14.00V) and adjust
//      VOLTAGE_DIVIDER_RATIO until readings match a calibrated DMM.
//   2. Current: Use a known load or power supply and adjust CURRENT_ZERO_POINT
//      and CURRENT_SCALE.
//   3. Temperature: Compare against a reference thermometer at room temperature
//      and at an elevated temperature if possible.

/// Voltage divider scaling factor.
/// Formula: V_battery = (ADC / 1023.0) * ADC_REFERENCE_V * VOLTAGE_DIVIDER_RATIO
/// Common example: 47k top + 10k bottom → ratio ≈ 5.70
constexpr float VOLTAGE_DIVIDER_RATIO = 5.70f;

/// Current sensor calibration (charge sensor on PIN_CURRENT_SENSE).
/// I = (rawADC - CURRENT_ZERO_POINT) * CURRENT_SCALE
///
/// Examples:
///   - ACS712-05B (bidirectional): zero ≈ 512, scale ≈ 0.0264 A/LSB
///   - Unidirectional shunt amp (0-5V = 0-10A): zero = 0, scale = 2.0
constexpr float CURRENT_ZERO_POINT = 512.0f; // raw ADC value at 0 A
constexpr float CURRENT_SCALE = 0.0264f;     // Amps per ADC count

/// Load current sensor uses the **same** calibration constants as the charge
/// sensor for simplicity. If your load sensor has different characteristics,
/// you can duplicate these constants with a _LOAD suffix.

/// NTC Temperature Sensor (Beta model)
/// Standard 10kΩ @ 25°C, Beta = 3950, 10k pull-up to 5V.
/// For higher accuracy consider switching to the Steinhart-Hart equation.
constexpr float NTC_NOMINAL_R = 10000.0f; // Resistance at 25 °C
constexpr float NTC_BETA = 3950.0f;
constexpr float NTC_NOMINAL_T = 25.0f;   // Reference temperature
constexpr float NTC_PULLUP_R = 10000.0f; // Pull-up resistor value

/// Number of ADC samples taken and averaged per sensor reading.
/// Higher values improve noise rejection at the cost of longer active time
/// (and thus slightly higher average power).
constexpr uint8_t ADC_SAMPLES = 16;

// =============================================================================
// CONTROL LOOP & PWM
// =============================================================================
// These values define the behavior of the P + limited-I controller.
// They have been tuned for typical small-to-medium charger hardware.
// Significant changes may require re-tuning of the slew limit as well.

/// PWM output range (0-255 for 8-bit Timer1)
constexpr uint16_t PWM_MAX_DUTY = 255;

/// Controller gains — tune these for your specific power stage.
/// KP_CURRENT / KP_VOLTAGE control responsiveness.
/// KI_CURRENT provides steady-state accuracy in current regulation only.
constexpr float KP_CURRENT = 0.8f;  // Proportional gain for current mode
constexpr float KI_CURRENT = 0.05f; // Integral gain for current mode (anti-windup applied)
constexpr float KP_VOLTAGE = 40.0f; // Proportional gain for voltage mode (pure P)

/// Slew rate limiter on PWM output. Prevents sudden changes that can cause
/// voltage spikes, audible noise, or stress on inductors/MOSFETs.
constexpr uint8_t PWM_SLEW_LIMIT = 8; // Maximum duty change per control cycle

// =============================================================================
// TIMING PARAMETERS
// =============================================================================
// All timing is expressed in wake-up cycles for simplicity.
// With SLEEP_INTERVAL_MS = 2000, one "minute" of real time is ~30 wake cycles.

/// Main control loop period. This is the interval between deep sleep periods.
/// Lower values increase responsiveness but raise average power consumption.
constexpr uint16_t SLEEP_INTERVAL_MS = 2000; // 2 seconds between control cycles

/// Derived value used for long-term timers (absorption minutes, float hours).
constexpr uint16_t WAKES_PER_MINUTE = 30; // 60000 / SLEEP_INTERVAL_MS

/// Deep-discharge pre-charge safety timeout (real minutes, converted with WAKES_PER_MINUTE).
constexpr uint16_t PRECHARGE_MAX_MINUTES = 30;

/// Number of consecutive safety violations required before latching a FAULT.
/// Provides basic debouncing against transient sensor noise.
constexpr uint8_t FAULT_DEBOUNCE_CYCLES = 3;

/// Throttle interval for non-critical EEPROM writes (protects flash endurance).
/// EEPROM cells have a typical endurance of 100,000 write cycles.
constexpr uint16_t EEPROM_SAVE_INTERVAL = 300; // ~10 minutes between throttled saves

// =============================================================================
// SERIAL USER INTERFACE
// =============================================================================

constexpr uint32_t SERIAL_BAUD = 115200;       // Serial speed for commands and telemetry
constexpr uint16_t STATUS_PRINT_INTERVAL = 15; // How often the compact CSV line is emitted

// =============================================================================
// EEPROM LAYOUT (robust with magic + CRC)
// =============================================================================
// The EepromData struct is versioned. When changing the layout, increment
// EEPROM_VERSION. Old data will be detected as invalid and reinitialized.

constexpr uint16_t EEPROM_MAGIC = 0xB177; // Magic number ("BM" in hex) for corruption detection
constexpr uint8_t EEPROM_VERSION = 2;     // Schema version. Bump on breaking changes.

/// Persistent data stored in EEPROM (first 16-20 bytes).
/// CRC8 is calculated over all fields except itself.
struct __attribute__((packed)) EepromData {
    uint16_t magic;                   // Must match EEPROM_MAGIC
    uint8_t version;                  // Must match EEPROM_VERSION
    uint8_t lastState;                // Last known ChargeState (for recovery)
    uint32_t chargedMilliAmpHours;    // Total energy put into the battery
    uint32_t dischargedMilliAmpHours; // Total energy taken out of the battery
    int16_t vCalOffset;               // Runtime voltage calibration offset (0.01 V units)
    int16_t iCalOffset;               // Runtime current calibration offset (0.01 A units)
    uint8_t crc8;                     // CRC-8 checksum for data integrity
};

// =============================================================================
// SAFETY & MISC
// =============================================================================

/// ADC reference voltage. Most Arduino boards use a 5V regulator.
/// Some advanced designs use the internal 1.1V reference for better resolution.
constexpr float ADC_REFERENCE_V = 5.0f;

/// Hardware watchdog timeout. The WDT is independent of the software loop
/// and will reset the MCU if it becomes unresponsive.
constexpr uint16_t WATCHDOG_TIMEOUT_MS = 8000; // 8 seconds

/// Runtime cal offsets are stored in 0.01-unit ticks and applied before safety.
constexpr float CAL_OFFSET_MAX_V = 0.50f;
constexpr float CAL_OFFSET_MAX_A = 0.50f;

// =============================================================================
// ENUMERATIONS
// =============================================================================

/// Charging states used by the main state machine.
/// The ASCII diagram in README.md and DESIGN.md shows the allowed transitions.
enum class ChargeState : uint8_t { INIT = 0, IDLE, PRECHARGE, BULK, ABSORPTION, FLOAT, FAULT, RECOVERY };

/// Fault codes. Multiple faults can theoretically be active (bitmask),
/// although the current implementation only reports one at a time.
enum class FaultCode : uint8_t {
    NONE = 0,
    OVER_VOLTAGE = 1 << 0,
    UNDER_VOLTAGE = 1 << 1,
    OVER_CURRENT = 1 << 2,
    OVER_TEMP = 1 << 3,
    UNDER_TEMP = 1 << 4,
    CHARGE_TIMEOUT = 1 << 5,
    SENSOR_FAULT = 1 << 6,
    MANUAL = 1 << 7
};

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

/// Returns the human-readable name of the currently selected battery profile.
/// Used in boot messages and the `dump` serial command.
inline const char *getProfileName() { return PROFILE_NAME; }

#endif // BATTERY_MANAGER_CONFIG_H
