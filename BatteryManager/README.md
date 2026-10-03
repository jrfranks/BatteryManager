# BatteryManager Arduino Sketch

AVR HAL and the charge-supervisor sketch. Open this folder in the Arduino IDE.

## Contents

- `BatteryManager.ino` — ADC, Timer1 PWM, sleep, watchdog, EEPROM, serial, optional OLED
- `Config.h` — the file you edit: pins, one chemistry profile, sensor scales you must measure
- `src/core/` — header-only charge algorithms. Arduino IDE 1.5+ compiles `src/` on its own

## Quick Usage

1. Open this folder in the Arduino IDE, or compile it from the repo root with `arduino-cli`.
2. Install the **Low-Power** library (rocketscream).
3. Measure the divider and the current sensor. Replace `VOLTAGE_DIVIDER_RATIO` and `CURRENT_SCALE` in `Config.h`. The checked-in numbers are examples.
4. Keep `BATTERY_PROFILE_LEAD_ACID_12V` unless the pack already has a BMS and a charger IC.
5. Upload to an Uno, Nano, or Pro Mini. The boot banner is **115200** baud. The USART is then turned off.

Pin names are in `Config.h`. This repo has no wiring diagram. Architecture and the power tradeoffs are in [DESIGN.md](../DESIGN.md). The root [README](../README.md) says what the host tests cover and what they do not.

## PlatformIO

```bash
pio run -e uno
pio test -e native
```

## Version

v1.1 — temperature compensation, coulomb counting, JSON telemetry, optional OLED. Sleep current is not a measured field of this version.
