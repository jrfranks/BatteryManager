#pragma once

#ifndef BATTERY_MANAGER_CRC8_H
#define BATTERY_MANAGER_CRC8_H

#include <stdint.h>

/// Dallas/Maxim CRC-8 (reflected polynomial 0x8C).
inline uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--) {
        uint8_t in = *data++;
        for (uint8_t i = 8; i; i--) {
            uint8_t mix = (crc ^ in) & 0x01;
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            in >>= 1;
        }
    }
    return crc;
}

#endif // BATTERY_MANAGER_CRC8_H
