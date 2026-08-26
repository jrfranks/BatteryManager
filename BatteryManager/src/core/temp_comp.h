#pragma once

#ifndef BATTERY_MANAGER_TEMP_COMP_H
#define BATTERY_MANAGER_TEMP_COMP_H

#include "../../Config.h"

/// Linear compensation: V_comp = V_base + (mV_per_C / 1000) * (T - 25).
/// Negative mV_per_C lowers charge voltage as temperature rises.
inline float compensateVoltage(float baseVoltage, float tempC, float mVPerC)
{
    float v = baseVoltage;
    if (mVPerC != 0.0f) {
        v = baseVoltage + (mVPerC / 1000.0f) * (tempC - 25.0f);
    }
    if (v > MAX_CHARGE_VOLTAGE) {
        v = MAX_CHARGE_VOLTAGE;
    }
    if (v < MIN_OPERATING_VOLTAGE) {
        v = MIN_OPERATING_VOLTAGE;
    }
    return v;
}

inline float compensateVoltage(float baseVoltage, float tempC)
{
    return compensateVoltage(baseVoltage, tempC, TEMP_COMP_mV_PER_C);
}

#endif // BATTERY_MANAGER_TEMP_COMP_H
