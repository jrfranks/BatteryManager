#pragma once

#include "charger_controller.h"
#include "persistence.h"
#include <string.h>

struct MockHal {
    float voltage = 13.0f;
    float current = 0.0f;
    float loadCurrent = 0.0f;
    float tempC = 25.0f;
    uint8_t pwm = 0;
    bool chargeEnable = false;
    uint8_t eepromMem[sizeof(EepromData)]{};
    uint32_t wakeCount = 0;
    uint32_t lastEepromSave = 0;
    bool masterEnable = true;
    unsigned saveCount = 0;
    unsigned loadCount = 0;

    static inline MockHal *inst = nullptr;

    MockHal()
    {
        memset(eepromMem, 0xFF, sizeof(eepromMem));
        inst = this;
    }

    ~MockHal()
    {
        if (inst == this) {
            inst = nullptr;
        }
    }

    static float readVoltage() { return inst->voltage; }
    static float readCurrent() { return inst->current; }
    static float readLoadCurrent() { return inst->loadCurrent; }
    static float readTempC() { return inst->tempC; }
    static uint8_t getPwm() { return inst->pwm; }
    static void setPwm(uint8_t d) { inst->pwm = d; }
    static void setChargeEnable(bool en) { inst->chargeEnable = en; }
    static void eepromLoad(EepromData *d)
    {
        inst->loadCount++;
        memcpy(d, inst->eepromMem, sizeof(EepromData));
    }
    static void eepromSave(const EepromData *d)
    {
        inst->saveCount++;
        memcpy(inst->eepromMem, d, sizeof(EepromData));
    }

    ChargerHal hal()
    {
        ChargerHal h{};
        h.readVoltage = readVoltage;
        h.readCurrent = readCurrent;
        h.readLoadCurrent = readLoadCurrent;
        h.readTempC = readTempC;
        h.getPwm = getPwm;
        h.setPwm = setPwm;
        h.setChargeEnable = setChargeEnable;
        h.eepromLoad = eepromLoad;
        h.eepromSave = eepromSave;
        h.wakeCount = &wakeCount;
        h.lastEepromSave = &lastEepromSave;
        h.masterEnable = &masterEnable;
        return h;
    }

    void plantEeprom(const EepromData &d) { memcpy(eepromMem, &d, sizeof(d)); }

    EepromData peekEeprom() const
    {
        EepromData d{};
        memcpy(&d, eepromMem, sizeof(d));
        return d;
    }
};

inline void plantValidEeprom(MockHal &mock, ChargeState lastState, uint32_t chargedmAh = 0, uint32_t dischargedmAh = 0,
  int16_t vCal = 0, int16_t iCal = 0)
{
    EepromData d{};
    eepromInitDefaults(d);
    d.lastState = static_cast<uint8_t>(lastState);
    d.chargedMilliAmpHours = chargedmAh;
    d.dischargedMilliAmpHours = dischargedmAh;
    d.vCalOffset = vCal;
    d.iCalOffset = iCal;
    eepromFinalizeCrc(d);
    mock.plantEeprom(d);
}

inline void runCycles(ChargerController &c, MockHal &mock, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i) {
        c.runCycle();
        mock.wakeCount++;
    }
}

inline void settle(ChargerController &c, MockHal &mock, uint32_t n = 24) { runCycles(c, mock, n); }
