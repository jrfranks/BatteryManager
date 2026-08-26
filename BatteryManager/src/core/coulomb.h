#pragma once

#ifndef BATTERY_MANAGER_COULOMB_H
#define BATTERY_MANAGER_COULOMB_H

#include "../../Config.h"

constexpr float COULOMB_DEADBAND_A = 0.02f;

inline void accumulateCoulomb(EepromData &eeprom, float chargeCurrent, float loadCurrent)
{
    float netCurrent = chargeCurrent - loadCurrent;
    if (netCurrent > COULOMB_DEADBAND_A) {
        float mAh = netCurrent * (SLEEP_INTERVAL_MS / 3600000.0f);
        eeprom.chargedMilliAmpHours += (uint32_t)(mAh * 1000.0f + 0.5f);
    } else if (netCurrent < -COULOMB_DEADBAND_A) {
        float mAh = -netCurrent * (SLEEP_INTERVAL_MS / 3600000.0f);
        eeprom.dischargedMilliAmpHours += (uint32_t)(mAh * 1000.0f + 0.5f);
    }
}

inline uint32_t milliAhToAh(uint32_t milliAh) { return milliAh / 1000UL; }

#endif // BATTERY_MANAGER_COULOMB_H
