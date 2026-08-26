#pragma once

#ifndef BATTERY_MANAGER_SAFETY_H
#define BATTERY_MANAGER_SAFETY_H

#include "../../Config.h"

inline FaultCode checkSafetyLimits(float v, float i, float t, ChargeState state)
{
    if (v > MAX_CHARGE_VOLTAGE) {
        return FaultCode::OVER_VOLTAGE;
    }
    if (v < MIN_OPERATING_VOLTAGE && state != ChargeState::INIT) {
        return FaultCode::UNDER_VOLTAGE;
    }
    if (i > MAX_CHARGE_CURRENT * 1.25f) {
        return FaultCode::OVER_CURRENT;
    }
    if (t > MAX_TEMP_C) {
        return FaultCode::OVER_TEMP;
    }
    if (t < MIN_TEMP_C && (state == ChargeState::BULK || state == ChargeState::ABSORPTION)) {
        return FaultCode::UNDER_TEMP;
    }
    return FaultCode::NONE;
}

#endif // BATTERY_MANAGER_SAFETY_H
