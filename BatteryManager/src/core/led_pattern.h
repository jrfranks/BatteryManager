#pragma once

#ifndef BATTERY_MANAGER_LED_PATTERN_H
#define BATTERY_MANAGER_LED_PATTERN_H

#include "../../Config.h"

inline bool ledOnForState(ChargeState s, uint32_t wakeCount)
{
    switch (s) {
    case ChargeState::IDLE:
        return (wakeCount % 8u) < 1u;
    case ChargeState::BULK:
        return (wakeCount % 3u) < 2u;
    case ChargeState::ABSORPTION:
        return (wakeCount % 4u) < 2u;
    case ChargeState::FLOAT:
        return (wakeCount % 6u) < 1u;
    case ChargeState::FAULT:
        return (wakeCount % 2u) != 0u;
    case ChargeState::PRECHARGE:
        return (wakeCount % 5u) < 1u;
    default:
        return false;
    }
}

#endif // BATTERY_MANAGER_LED_PATTERN_H
