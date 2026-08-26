# BatteryManager host tests

Unity tests run on the build machine via PlatformIO native environments. They do not flash a board and do not talk to real ADCs, EEPROM, or PWM.

```bash
pio test -e native
pio test -e native_lifepo4
pio test -e native_liion
```

## Layout

| Path | Role |
|------|------|
| `test/test_*.cpp` | One file per domain (`crc8`, `sensor_math`, `temp_comp`, `control_law`, `coulomb`, `safety`, `fsm`, `eeprom`, `led`, `serial`, `telemetry`, `charger_cycle`) |
| `test/test_main.cpp` | Unity runner (`RUN_TEST` registration) |
| `test/support/` | Host stubs (`Arduino.h`) and `MockHal` |

`native_lifepo4` / `native_liion` compile the same sources with `-DBATTERY_PROFILE_*`. Extra chemistry cases live behind `#ifdef` in the same files.

## Adding a case

1. Put a `void test_your_name(void)` in the matching `test/test_*.cpp` (or add a new `test_*.cpp`).
2. Register it with `RUN_TEST(test_your_name);` inside that file's `run_*_tests()` function, and call that runner from `test_main.cpp` if you added a new file.
3. Include `Config.h` and assert against those constants — do not duplicate profile voltages/currents as magic numbers.
4. Use `TEST_ASSERT_FLOAT_WITHIN` (about `1e-4` V/A; NTC about `0.05` °C).
5. Drive `runCycles()` long enough for the IIR (`α = 0.25`) and `FAULT_DEBOUNCE_CYCLES` to settle. One `runCycle()` rarely completes an FSM jump from a large voltage step.

## How mocks work

`MockHal` implements `ChargerHal` with function pointers:

- Scripted `voltage` / `current` / `loadCurrent` / `tempC`
- Captured `pwm` and `chargeEnable`
- RAM EEPROM (`eepromMem`), `saveCount` / `loadCount`
- `wakeCount`, `lastEepromSave`, `masterEnable` matching `loop()`

Callbacks go through a process-wide `MockHal::inst`. Construct only one live `MockHal` per test (or destroy the previous one before creating another).

`runCycles(controller, mock, n)` calls `runCycle()` then increments `wakeCount`, the same order as `loop()`.

Virgin EEPROM is `0xFF`. Use `plantValidEeprom()` to restore a `ChargeState` and CRC-valid blob.

## What is not tested here

AVR registers, `LowPower.powerDown`, WDT, ADC burst/prescaler, OLED I2C, and real analog hardware stay compile-only on the Uno environments.
