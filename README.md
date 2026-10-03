# BatteryManager

AVR charge-supervisor firmware for an ATmega328P (Arduino Uno, Nano, or Pro Mini). A CC/CV state machine drives Timer1 PWM and a charge-enable pin. Debounced voltage, current, and temperature checks force both outputs off. This is not a BMS and not a charger IC. A lithium pack still needs both.

Host Unity tests exercise the charge state machine, safety thresholds, control law, coulomb counter, CRC, calibration math, and telemetry with a mocked HAL. GitHub Actions compiles the Uno and Pro Mini sketches (lead-acid, LiFePO4, and Li-ion profiles) and runs those host tests. ADC burst, Timer1 PWM, `powerDown`, the watchdog, OLED I2C, and the power stage are compile-checked only. This repo has no measured sleep-current number and no schematic.

The sleep path is `PRR` and `DIDR0` before `LowPower.powerDown`, a divider power gate, USART off after the boot banner, Timer1 stopped when idle, and unused pins as `INPUT_PULLUP`. `BOD_OFF` during sleep is a documented tradeoff, not a microamp result.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
![Platform](https://img.shields.io/badge/platform-AVR%20(Arduino)-blue)
![Build](https://github.com/jrfranks/BatteryManager/actions/workflows/arduino-ci.yml/badge.svg)

---

## Features

- CC/CV state machine (bulk, absorption, float, fault, recovery) with debounced threshold trips
- Sleep path: `PRR`, `DIDR0`, `BOD_OFF`, serial off after boot, Timer1 off when idle
- Temperature-compensated absorption and float targets
- Coulomb counting when a load-side current sensor is wired
- Optional JSON telemetry and SSD1306 OLED (`ENABLE_OLED`, off by default)
- IIR-filtered sensors and a P+limited-I law on Timer1
- EEPROM record (Ah, calibration, last state, CRC-8) with a write throttle
- Serial calibration commands, off after boot unless `SERIAL_DEBUG_ALWAYS`
- Watchdog (8 s) reset every wake
- Arduino IDE sketch and PlatformIO, same sources

---

## Repository Layout

```
LICENSE
BatteryManager/                 # Arduino sketch
├── BatteryManager.ino          # HAL: ADC, Timer1, sleep, EEPROM, watchdog
├── Config.h                    # pins, chemistry, calibration (edit this)
├── src/core/                   # header-only charge algorithms
└── README.md
test/                           # native Unity suite
DESIGN.md                       # why the power and control choices were made
platformio.ini
.github/workflows/arduino-ci.yml
```

There is no wiring diagram in this repo. Pin names are in `BatteryManager/Config.h`. Confirm them against the board before connecting a pack.

---

## Quick Start (Arduino IDE)

1. Open the `BatteryManager/` folder as a sketch.
2. Install the "Low-Power" library by rocketscream.
3. Measure the divider and the current sensor. Write those numbers into `Config.h`. The checked-in `5.70` and `0.0264` are ACS712-style examples, not a built power stage.
4. Leave the default lead-acid profile unless an external BMS is already on the pack.
5. Upload. Serial banner is 115200 baud, then the USART is powered down.

---

## Quick Start (PlatformIO)

```bash
pio run -e uno          # build
pio upload -e uno
pio device monitor
```

`platformio.ini` builds the same sketch sources.

---

## Testing

```bash
pio test -e native            # lead-acid profile
pio test -e native_lifepo4
pio test -e native_liion
```

Host coverage: CRC-8 and EEPROM validation, ADC scaling and NTC math, IIR and calibration offsets, temperature compensation, P+limited-I, coulomb counting, safety limits and debounce, charge FSM, EEPROM wear throttle, LED patterns, serial command parser, CSV/JSON telemetry, and HAL-mocked `runCycle()`.

Not executed on the host: AVR ADC burst and prescaler, Timer1 PWM, `LowPower.powerDown`, watchdog, OLED I2C, unused-pin pull-ups, sensor hardware, and the power stage.

See [test/README.md](test/README.md).

---

## Safety

Software thresholds are not a hardware kill. `checkSafetyLimits()` runs before `applyControlOutputs()`. A trip forces PWM to 0 and charge-enable low, and latches `FAULT` after `FAULT_DEBOUNCE_CYCLES`. There is no series MOSFET, fuse, or pack-protection IC in this firmware.

Lead-acid is the default profile. LiFePO4 and Li-ion constants in `Config.h` are generic starting points (Li-ion 4S absorption is 16.8 V, float is 16.0 V). Do not use either lithium profile without a charger IC and a BMS. Do not float a lithium cell unless the cell maker says to.

Chemistry limits, the `BOD_OFF` tradeoff, and the control law are in [DESIGN.md](DESIGN.md).

---

## License

MIT © 2023–2026 John R. Franks and contributors.
