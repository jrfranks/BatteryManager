#pragma once

#ifndef BATTERY_MANAGER_TELEMETRY_H
#define BATTERY_MANAGER_TELEMETRY_H

#include "../../Config.h"
#include "charger_controller.h"
#include <stddef.h>
#include <string.h>

#if defined(ARDUINO) && defined(__AVR__)
#include <avr/pgmspace.h>
#include <stdlib.h>
#define TELEM_STR(s) PSTR(s)
#else
#include <stdio.h>
#define TELEM_STR(s) (s)
#endif

inline const char *chargeStateName(ChargeState s)
{
    switch (s) {
    case ChargeState::INIT:
        return TELEM_STR("INIT");
    case ChargeState::IDLE:
        return TELEM_STR("IDLE");
    case ChargeState::PRECHARGE:
        return TELEM_STR("PRECHARGE");
    case ChargeState::BULK:
        return TELEM_STR("BULK");
    case ChargeState::ABSORPTION:
        return TELEM_STR("ABSORPTION");
    case ChargeState::FLOAT:
        return TELEM_STR("FLOAT");
    case ChargeState::FAULT:
        return TELEM_STR("FAULT");
    case ChargeState::RECOVERY:
        return TELEM_STR("RECOVERY");
    default:
        return TELEM_STR("?");
    }
}

inline const char *faultName(FaultCode f)
{
    uint8_t bits = static_cast<uint8_t>(f);
    if (bits == 0) {
        return TELEM_STR("None");
    }
    if (bits & static_cast<uint8_t>(FaultCode::OVER_VOLTAGE)) {
        return TELEM_STR("OverVoltage");
    }
    if (bits & static_cast<uint8_t>(FaultCode::UNDER_VOLTAGE)) {
        return TELEM_STR("UnderVoltage");
    }
    if (bits & static_cast<uint8_t>(FaultCode::OVER_CURRENT)) {
        return TELEM_STR("OverCurrent");
    }
    if (bits & static_cast<uint8_t>(FaultCode::OVER_TEMP)) {
        return TELEM_STR("OverTemp");
    }
    if (bits & static_cast<uint8_t>(FaultCode::UNDER_TEMP)) {
        return TELEM_STR("UnderTemp");
    }
    if (bits & static_cast<uint8_t>(FaultCode::CHARGE_TIMEOUT)) {
        return TELEM_STR("Timeout");
    }
    if (bits & static_cast<uint8_t>(FaultCode::SENSOR_FAULT)) {
        return TELEM_STR("Sensor");
    }
    if (bits & static_cast<uint8_t>(FaultCode::MANUAL)) {
        return TELEM_STR("Manual");
    }
    return TELEM_STR("Unknown");
}

namespace telem_detail
{

inline void putChar(char *&p, char *end, char c)
{
    if (p == nullptr || end == nullptr || p + 1 >= end) {
        return;
    }
    *p++ = c;
    *p = '\0';
}

inline void putStr(char *&p, char *end, const char *s)
{
    if (s == nullptr) {
        return;
    }
    while (*s) {
        putChar(p, end, *s++);
    }
}

#if defined(ARDUINO) && defined(__AVR__)
inline void putP(char *&p, char *end, const char *progmemStr)
{
    if (progmemStr == nullptr) {
        return;
    }
    char c;
    while ((c = static_cast<char>(pgm_read_byte(progmemStr++))) != '\0') {
        putChar(p, end, c);
    }
}
#else
inline void putP(char *&p, char *end, const char *s) { putStr(p, end, s); }
#endif

inline void putUInt(char *&p, char *end, uint32_t v)
{
    char tmp[11];
    uint8_t n = 0;
    if (v == 0) {
        putChar(p, end, '0');
        return;
    }
    while (v && n < sizeof(tmp)) {
        tmp[n++] = static_cast<char>('0' + (v % 10u));
        v /= 10u;
    }
    while (n--) {
        putChar(p, end, tmp[n]);
    }
}

inline void putFloat(char *&p, char *end, float v, uint8_t decimals)
{
    char tmp[24];
#if defined(ARDUINO) && defined(__AVR__)
    dtostrf(v, 0, decimals, tmp);
    putStr(p, end, tmp);
#else
    int n = snprintf(tmp, sizeof(tmp), "%.*f", static_cast<int>(decimals), static_cast<double>(v));
    if (n < 0) {
        return;
    }
    putStr(p, end, tmp);
#endif
}

} // namespace telem_detail

inline void formatTelemetryCsv(char *buf, size_t len, const ChargerController &c)
{
    if (buf == nullptr || len == 0) {
        return;
    }
    buf[0] = '\0';
    char *p = buf;
    char *end = buf + len;
    using namespace telem_detail;
    putP(p, end, TELEM_STR("BM,"));
    putUInt(p, end, static_cast<uint32_t>(c.getState()));
    putChar(p, end, ',');
    putFloat(p, end, c.getVoltage(), 2);
    putChar(p, end, ',');
    putFloat(p, end, c.getCurrent(), 2);
    putChar(p, end, ',');
    putFloat(p, end, c.getLoadCurrent(), 2);
    putChar(p, end, ',');
    putFloat(p, end, c.getNetCurrent(), 2);
    putChar(p, end, ',');
    putFloat(p, end, c.getTemp(), 1);
    putChar(p, end, ',');
    putUInt(p, end, c.getPWM());
    putChar(p, end, ',');
    putUInt(p, end, static_cast<uint8_t>(c.getFault()));
    putChar(p, end, ',');
    putUInt(p, end, c.getChargedAh());
    putChar(p, end, ',');
    putUInt(p, end, c.getDischargedAh());
}

inline void formatTelemetryJson(char *buf, size_t len, const ChargerController &c)
{
    if (buf == nullptr || len == 0) {
        return;
    }
    buf[0] = '\0';
    char *p = buf;
    char *end = buf + len;
    using namespace telem_detail;
    putP(p, end, TELEM_STR("{\"state\":\""));
    putP(p, end, chargeStateName(c.getState()));
    putP(p, end, TELEM_STR("\",\"v\":"));
    putFloat(p, end, c.getVoltage(), 3);
    putP(p, end, TELEM_STR(",\"i_charge\":"));
    putFloat(p, end, c.getCurrent(), 3);
    putP(p, end, TELEM_STR(",\"i_load\":"));
    putFloat(p, end, c.getLoadCurrent(), 3);
    putP(p, end, TELEM_STR(",\"i_net\":"));
    putFloat(p, end, c.getNetCurrent(), 3);
    putP(p, end, TELEM_STR(",\"temp\":"));
    putFloat(p, end, c.getTemp(), 1);
    putP(p, end, TELEM_STR(",\"pwm\":"));
    putUInt(p, end, c.getPWM());
    putP(p, end, TELEM_STR(",\"target_v\":"));
    putFloat(p, end, c.getTargetVoltage(), 3);
    putP(p, end, TELEM_STR(",\"comp_absorb_v\":"));
    putFloat(p, end, c.getCompensatedAbsorptionV(), 3);
    putP(p, end, TELEM_STR(",\"comp_float_v\":"));
    putFloat(p, end, c.getCompensatedFloatV(), 3);
    putP(p, end, TELEM_STR(",\"charged_ah\":"));
    putUInt(p, end, c.getChargedAh());
    putP(p, end, TELEM_STR(",\"discharged_ah\":"));
    putUInt(p, end, c.getDischargedAh());
    putP(p, end, TELEM_STR(",\"net_ah\":"));
    putUInt(p, end, c.getNetAh());
    putChar(p, end, '}');
}

#endif // BATTERY_MANAGER_TELEMETRY_H
