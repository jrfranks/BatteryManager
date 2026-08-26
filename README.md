# BatteryManager

**Elegant, robust, low-power battery charge controller for Arduino AVR**

A production-quality firmware for safely charging lead-acid, LiFePO4, and Li-ion battery packs using a simple Arduino (Uno/Nano/Pro Mini) + low-power techniques. Designed for solar, UPS, and bench applications where reliability and minimal quiescent current matter.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
![Platform](https://img.shields.io/badge/platform-AVR%20(Arduino)-blue)
![RAM](https://img.shields.io/badge/RAM-~504%20bytes-brightgreen)
![Flash](https://img.shields.io/badge/Flash-~16.6%20kB-brightgreen)
![Build](https://github.com/jrfranks/BatteryManager/actions/workflows/arduino-ci.yml/badge.svg)

---

## Features

- **Chemistry-aware CC/CV state machine** with hard safety interlocks
- **Deep-sleep operation** (~2 s cycles via Low-Power library)
- **Ultra-low-power techniques**: conditional peripherals (Serial + Timer1), PRR/DIDR0 shutdown, BOD_OFF, fast ADC bursts, throttled EEPROM writes, all unused pins as INPUT_PULLUP
- **Temperature compensated charging** (absorption & float voltages)
- **True coulomb counting** with optional load-side current sensor (separate Ah in/out)
- **Richer telemetry** (JSON output) + optional SSD1306 OLED support
- **Robust filtered sensors** + P+I PWM control on Timer1 (timer stopped when idle)
- **EEPROM persistence** (Ah counter, calibration, last state + CRC) with minimal write wear
- **Excellent serial UI** with live calibration commands (disabled by default after boot for power)
- **Non-blocking multi-pattern status LED**
- **Watchdog + brown-out recovery**
- **First-class support** for both classic Arduino IDE **and** PlatformIO

---

## Repository Layout

```
LICENSE                         # MIT license (canonical)
BatteryManager/                 # Arduino sketch (open this folder in Arduino IDE)
├── BatteryManager.ino          # HAL + setup()/loop()
├── Config.h
├── src/core/                   # Host-testable charge algorithms (header-only)
└── README.md                   # Sketch-specific notes
test/                           # Native Unity suite (PlatformIO)
DESIGN.md                       # Full design notes, architecture decisions, and theoretical justifications (power, control, peripherals, etc.)
platformio.ini                  # AVR builds + native unit-test environments
.github/workflows/arduino-ci.yml
.gitignore                      # Enhanced for Arduino + PlatformIO
```

Full user documentation, wiring diagrams, calibration steps, serial commands, and safety guidance live in **[BatteryManager/README.md](BatteryManager/README.md)**.

---

## Quick Start (Arduino IDE)

1. Open the `BatteryManager/` folder as a sketch in the Arduino IDE.
2. Install the "Low-Power" library by rocketscream.
3. Edit `Config.h` to match your battery chemistry and sensor hardware.
4. Upload. Open Serial Monitor at 115200 baud.

See the detailed guide inside the `BatteryManager/` folder.

---

## Quick Start (PlatformIO)

```bash
pio run -e uno          # build
pio upload -e uno
pio device monitor
```

The `platformio.ini` is configured to build the exact same sources as the Arduino sketch.

---

## Testing

Host-native Unity tests exercise the safety-critical algorithms without hardware:

```bash
pio test -e native            # default lead-acid profile
pio test -e native_lifepo4
pio test -e native_liion
```

Covered on the host: CRC-8 / EEPROM validation, ADC scaling and NTC math, IIR and cal offsets, temperature compensation, P+limited-I control law, coulomb counting, safety limits and debounce, charge FSM, EEPROM wear throttle, LED patterns, serial command parser, CSV/JSON telemetry, and HAL-mocked `runCycle()` integration.

Still hardware-only (compile-checked, not executed on the host): AVR ADC burst / prescaler, Timer1 PWM, `LowPower.powerDown`, watchdog, OLED I2C, unused-pin pull-ups, and real sensor/power-stage behavior.

See [test/README.md](test/README.md) for how to add a case.

---

## Key Features & Capabilities

The firmware includes the following major capabilities:

- Full CC/CV state machine with safety interlocks (Bulk / Absorption / Float / Fault handling)
- Extremely low-power design (sub-20 µA sleep current with aggressive peripheral power-down)
- Robust sensor filtering and P+I control loop
- Runtime calibration via serial
- EEPROM-backed persistence with wear mitigation
- First-class PlatformIO + GitHub Actions support

---

## Safety & Philosophy

This is a **software-defined charge controller**, not a replacement for a proper BMS on lithium batteries. Always use hardware protection (fuses, dedicated protection boards, TVS).

The design deliberately favours **robustness, readability, and low power** over maximum performance.

For the full theoretical background, justifications for power optimizations, control strategy, peripheral management, and all other implementation choices, see **[DESIGN.md](DESIGN.md)** (which now contains the consolidated reasoning).

Architectural decisions and history are covered in **[DESIGN.md](DESIGN.md)**.

---

## Contributing & Future

Pull requests that improve safety, add support for new chemistries, or enhance the CI are very welcome.

See the sketch README for a list of possible extensions (OLED, second current sensor, true MPPT, etc.).

---

## License

MIT © 2023–2026 John R. Franks and contributors.

---
