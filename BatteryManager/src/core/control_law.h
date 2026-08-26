#pragma once

#ifndef BATTERY_MANAGER_CONTROL_LAW_H
#define BATTERY_MANAGER_CONTROL_LAW_H

#include "../../Config.h"

inline float clampFloat(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

inline int16_t clampI16(int16_t v, int16_t lo, int16_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

struct ControlLawOutput {
    uint8_t duty;
    bool enable;
    float pwmIntegral;
};

inline ControlLawOutput applyControlLaw(ChargeState state, bool masterEnable, float v, float i, float targetVoltage,
  float targetCurrent, uint8_t currentDuty, float pwmIntegral)
{
    ControlLawOutput out{};
    out.pwmIntegral = pwmIntegral;
    out.duty = 0;
    out.enable = false;

    if (!masterEnable || state == ChargeState::FAULT || state == ChargeState::IDLE) {
        return out;
    }

    const bool voltageMode = (state == ChargeState::ABSORPTION || state == ChargeState::FLOAT);
    float error;
    float kp;
    float ki;
    if (voltageMode) {
        error = targetVoltage - v;
        kp = KP_VOLTAGE;
        ki = 0.0f;
    } else {
        error = targetCurrent - i;
        kp = KP_CURRENT;
        ki = KI_CURRENT;
    }

    pwmIntegral += error * ki;
    pwmIntegral = clampFloat(pwmIntegral, -30.0f, 30.0f);

    float newDutyF = (currentDuty * 0.6f) + (error * kp) + pwmIntegral;
    int16_t newDuty = (int16_t)newDutyF;
    int16_t current = (int16_t)currentDuty;
    int16_t slew = (int16_t)PWM_SLEW_LIMIT;
    int16_t delta = clampI16((int16_t)(newDuty - current), (int16_t)(-slew), slew);
    uint8_t duty = (uint8_t)clampI16((int16_t)(current + delta), 0, (int16_t)PWM_MAX_DUTY);

    out.duty = duty;
    out.enable = (duty > 5) || (state == ChargeState::PRECHARGE);
    out.pwmIntegral = pwmIntegral;
    return out;
}

#endif // BATTERY_MANAGER_CONTROL_LAW_H
