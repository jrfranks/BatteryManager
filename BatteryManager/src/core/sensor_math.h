#pragma once

#ifndef BATTERY_MANAGER_SENSOR_MATH_H
#define BATTERY_MANAGER_SENSOR_MATH_H

#include "../../Config.h"
#include <math.h>

inline float adcToVoltage(float raw) { return (raw * ADC_REFERENCE_V / 1023.0f) * VOLTAGE_DIVIDER_RATIO; }

/// Charge/load current. Negative values are clamped to 0 (unidirectional).
inline float adcToCurrent(float raw)
{
    float amps = (raw - CURRENT_ZERO_POINT) * CURRENT_SCALE;
    if (amps < 0.0f) {
        amps = 0.0f;
    }
    return amps;
}

/// Beta-model NTC. Open/short (or r <= 0) returns 99.9 °C to trip OVER_TEMP.
inline float ntcRawToTempC(float raw)
{
    float v = (raw * ADC_REFERENCE_V / 1023.0f);
    if (v >= ADC_REFERENCE_V - 0.02f || v < 0.02f) {
        return 99.9f;
    }

    float r = (v * NTC_PULLUP_R) / (ADC_REFERENCE_V - v);
    if (r <= 0.0f) {
        return 99.9f;
    }

    float invT = (1.0f / (NTC_NOMINAL_T + 273.15f)) + (1.0f / NTC_BETA) * log(r / NTC_NOMINAL_R);
    return (1.0f / invT) - 273.15f;
}

inline float iirFilter(float y, float x) { return y * 0.75f + x * 0.25f; }

/// Runtime cal offset is stored in 0.01-unit ticks.
inline float applyCalOffset(float filtered, int16_t offsetTicks) { return filtered + (offsetTicks * 0.01f); }

#endif // BATTERY_MANAGER_SENSOR_MATH_H
