#pragma once

#ifndef BATTERY_MANAGER_PERSISTENCE_H
#define BATTERY_MANAGER_PERSISTENCE_H

#include "../../Config.h"
#include "crc8.h"
#include <stddef.h>

inline uint8_t eepromCrc(const EepromData &e)
{
    return crc8(reinterpret_cast<const uint8_t *>(&e), static_cast<uint8_t>(offsetof(EepromData, crc8)));
}

inline bool eepromIsValid(const EepromData &e)
{
    return e.magic == EEPROM_MAGIC && e.version == EEPROM_VERSION && eepromCrc(e) == e.crc8;
}

inline void eepromInitDefaults(EepromData &e)
{
    e.magic = EEPROM_MAGIC;
    e.version = EEPROM_VERSION;
    e.lastState = static_cast<uint8_t>(ChargeState::IDLE);
    e.chargedMilliAmpHours = 0;
    e.dischargedMilliAmpHours = 0;
    e.vCalOffset = 0;
    e.iCalOffset = 0;
    e.crc8 = 0;
}

inline void eepromFinalizeCrc(EepromData &e) { e.crc8 = eepromCrc(e); }

inline void persistenceLoad(EepromData &e, void (*load)(EepromData *), void (*save)(const EepromData *))
{
    load(&e);
    if (!eepromIsValid(e)) {
        eepromInitDefaults(e);
        eepromFinalizeCrc(e);
        save(&e);
    }
}

inline void persistenceSave(EepromData &e, void (*save)(const EepromData *))
{
    eepromFinalizeCrc(e);
    save(&e);
}

#endif // BATTERY_MANAGER_PERSISTENCE_H
