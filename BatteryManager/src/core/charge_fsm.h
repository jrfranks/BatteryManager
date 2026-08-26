#pragma once

#ifndef BATTERY_MANAGER_CHARGE_FSM_H
#define BATTERY_MANAGER_CHARGE_FSM_H

#include "../../Config.h"

inline uint32_t prechargeTimeoutWakes() { return (uint32_t)PRECHARGE_MAX_MINUTES * (uint32_t)WAKES_PER_MINUTE; }

inline uint32_t absorptionTimeoutWakes() { return (uint32_t)ABSORPTION_MAX_MINUTES * (uint32_t)WAKES_PER_MINUTE; }

/// Force IDLE on restore only when V is above absorption and near the hard cap.
inline float restoreForceIdleVoltage()
{
    float cap = MAX_CHARGE_VOLTAGE - 0.3f;
    if (cap < ABSORPTION_VOLTAGE) {
        cap = ABSORPTION_VOLTAGE + 0.05f;
    }
    return cap;
}

inline ChargeState nextIdleState(float v, float tempC, bool masterEnable)
{
    if (!masterEnable) {
        return ChargeState::IDLE;
    }
    if (tempC < MIN_TEMP_C || tempC > MAX_TEMP_C) {
        return ChargeState::IDLE;
    }
    if (v < PRECHARGE_VOLTAGE) {
        return ChargeState::PRECHARGE;
    }
    if (v < RECHARGE_VOLTAGE) {
        return ChargeState::BULK;
    }
    return ChargeState::IDLE;
}

inline bool prechargeExitToBulk(float v) { return v > PRECHARGE_VOLTAGE + 0.3f; }

inline bool prechargeTimedOut(uint32_t prechargeTicks) { return prechargeTicks >= prechargeTimeoutWakes(); }

inline bool bulkEnterAbsorption(float v, float i, float compensatedAbs)
{
    return v >= (compensatedAbs - 0.15f) && i < (MAX_CHARGE_CURRENT * 0.9f);
}

inline bool absorptionEnterFloat(float i, uint32_t absorptionMinutes)
{
    return i < ABSORPTION_EXIT_CURRENT || absorptionMinutes >= absorptionTimeoutWakes();
}

inline bool floatRestartBulk(float v) { return v < RECHARGE_VOLTAGE; }

inline bool isRestorableChargeState(ChargeState s) { return s >= ChargeState::IDLE && s <= ChargeState::FLOAT; }

inline bool isCriticalTransition(ChargeState from, ChargeState to)
{
    return (to == ChargeState::FAULT) || (to == ChargeState::BULK) || (to == ChargeState::ABSORPTION) ||
           (from == ChargeState::BULK && to == ChargeState::IDLE);
}

inline uint64_t floatRechargeWakeThreshold()
{
    return (uint64_t)FLOAT_RECHARGE_HOURS * 60ULL * (uint64_t)WAKES_PER_MINUTE;
}

#endif // BATTERY_MANAGER_CHARGE_FSM_H
