#pragma once

#ifndef BATTERY_MANAGER_SERIAL_PARSE_H
#define BATTERY_MANAGER_SERIAL_PARSE_H

#include "charger_controller.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t SERIAL_CMD_BUF_SIZE = 32;

enum class Command : uint8_t { None = 0, Help, Status, Start, Stop, Reset, Dump, Json, CalV, CalI, Unknown };

struct ParsedCommand {
    Command cmd;
    float value;
};

inline bool parseFiniteFloat(const char *s, float &out)
{
    if (s == nullptr || *s == '\0') {
        return false;
    }
    char *end = nullptr;
    float v = static_cast<float>(strtod(s, &end));
    if (end == s || *end != '\0') {
        return false;
    }
    if (!isfinite(v)) {
        return false;
    }
    out = v;
    return true;
}

inline ParsedCommand parseCommandLine(const char *buf)
{
    ParsedCommand r{};
    r.cmd = Command::None;
    r.value = 0.0f;
    if (buf == nullptr || buf[0] == '\0') {
        return r;
    }
    if (strcmp(buf, "help") == 0 || strcmp(buf, "?") == 0) {
        r.cmd = Command::Help;
    } else if (strcmp(buf, "status") == 0) {
        r.cmd = Command::Status;
    } else if (strcmp(buf, "start") == 0) {
        r.cmd = Command::Start;
    } else if (strcmp(buf, "stop") == 0) {
        r.cmd = Command::Stop;
    } else if (strcmp(buf, "reset") == 0) {
        r.cmd = Command::Reset;
    } else if (strcmp(buf, "dump") == 0) {
        r.cmd = Command::Dump;
    } else if (strcmp(buf, "json") == 0) {
        r.cmd = Command::Json;
    } else if (strncmp(buf, "cal v=", 6) == 0) {
        if (parseFiniteFloat(buf + 6, r.value)) {
            r.cmd = Command::CalV;
        } else {
            r.cmd = Command::Unknown;
        }
    } else if (strncmp(buf, "cal i=", 6) == 0) {
        if (parseFiniteFloat(buf + 6, r.value)) {
            r.cmd = Command::CalI;
        } else {
            r.cmd = Command::Unknown;
        }
    } else {
        r.cmd = Command::Unknown;
    }
    return r;
}

struct SerialCmdBuffer {
    char buf[SERIAL_CMD_BUF_SIZE];
    uint8_t idx;

    SerialCmdBuffer() : idx(0) { buf[0] = '\0'; }

    enum class FeedResult : uint8_t { Pending, Line };

    FeedResult feed(char c, ParsedCommand &parsed)
    {
        if (c == '\n' || c == '\r') {
            buf[idx] = '\0';
            idx = 0;
            parsed = parseCommandLine(buf);
            return FeedResult::Line;
        }
        if (idx < SERIAL_CMD_BUF_SIZE - 1) {
            buf[idx++] = c;
        }
        return FeedResult::Pending;
    }
};

inline void executeCommand(const ParsedCommand &parsed, ChargerController &charger)
{
    switch (parsed.cmd) {
    case Command::Start:
        charger.setMasterEnable(true);
        break;
    case Command::Stop:
        charger.setMasterEnable(false);
        charger.forceOutputsOff();
        break;
    case Command::Reset:
        charger.requestResetFault();
        break;
    case Command::CalV:
        charger.adjustVoltageCal(parsed.value);
        break;
    case Command::CalI:
        charger.adjustCurrentCal(parsed.value);
        break;
    default:
        break;
    }
}

#endif // BATTERY_MANAGER_SERIAL_PARSE_H
