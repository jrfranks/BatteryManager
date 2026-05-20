# BatteryManager Arduino Sketch

This folder contains the complete, ready-to-use Arduino sketch for the BatteryManager low-power battery charge controller.

## Contents

- `BatteryManager.ino` — Main firmware (setup/loop + full `ChargerController` implementation)
- `Config.h` — **The only file you normally need to edit** (pins, battery profile, calibration constants, safety limits)

## Quick Usage

1. Open this entire folder in the Arduino IDE (or use `arduino-cli` from the parent directory).
2. Install the **Low-Power** library (by rocketscream) via Library Manager.
3. Adjust your hardware settings in `Config.h` (especially `VOLTAGE_DIVIDER_RATIO`, current sensor scale, and chosen `#define BATTERY_PROFILE_*`).
4. Upload to an Arduino Uno, Nano, or Pro Mini.
5. Open the Serial Monitor at **115200 baud** and type `help`.

## Full Documentation

All wiring diagrams, state machine explanation, calibration procedure, serial command reference, safety notes, and troubleshooting live in the **[top-level README.md](../README.md)** of the repository.

For the full theoretical background and justifications behind the ultra-low-power techniques (PRR/DIDR0, BOD_OFF, conditional peripherals, fast ADC, reduced EEPROM writes, etc.), control strategy, and implementation choices, see **[DESIGN.md](../DESIGN.md)** (the consolidated architecture + reasoning document).

## PlatformIO Users

The project root contains a `platformio.ini` that builds this sketch directly:

```bash
pio run -e uno
```

## Version

Current version: v1.1 (temperature compensation, coulomb counting with load sensor, JSON telemetry, optional OLED)

---
