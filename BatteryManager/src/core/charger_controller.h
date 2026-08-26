#pragma once

#ifndef BATTERY_MANAGER_CHARGER_CONTROLLER_H
#define BATTERY_MANAGER_CHARGER_CONTROLLER_H

#include "../../Config.h"
#include "charge_fsm.h"
#include "control_law.h"
#include "coulomb.h"
#include "persistence.h"
#include "safety.h"
#include "sensor_math.h"
#include "temp_comp.h"
#include <math.h>

/// All function pointers and data pointers must be non-null for the controller lifetime.
struct ChargerHal {
    float (*readVoltage)();
    float (*readCurrent)();
    float (*readLoadCurrent)();
    float (*readTempC)();
    uint8_t (*getPwm)();
    void (*setPwm)(uint8_t);
    void (*setChargeEnable)(bool);
    void (*eepromLoad)(EepromData *);
    void (*eepromSave)(const EepromData *);
    uint32_t *wakeCount;
    uint32_t *lastEepromSave;
    bool *masterEnable;
};

inline int16_t roundToCalTicks(float delta)
{
    float x = delta * 100.0f;
    x = (x >= 0.0f) ? (x + 0.5f) : (x - 0.5f);
    return static_cast<int16_t>(x);
}

class ChargerController
{
  public:
    explicit ChargerController(const ChargerHal &h)
        : hal(h), state(ChargeState::INIT), filteredV(0.0f), filteredI(0.0f), filteredLoadI(0.0f), filteredT(25.0f),
          targetCurrent(0.0f), targetVoltage(0.0f), compensatedAbsorptionV(0.0f), compensatedFloatV(0.0f),
          pwmIntegral(0.0f), absorptionMinutes(0), prechargeTicks(0), floatHours(0), faultReason(FaultCode::NONE),
          consecutiveFaults(0), eeprom{}
    {
    }

    void begin()
    {
        loadFromEEPROM();

        filteredV = hal.readVoltage();
        filteredI = hal.readCurrent();
        filteredT = hal.readTempC();

        ChargeState restored = static_cast<ChargeState>(eeprom.lastState);
        if (isRestorableChargeState(restored)) {
            state = restored;
        } else {
            state = ChargeState::INIT;
        }

        if (filteredV > restoreForceIdleVoltage()) {
            state = ChargeState::IDLE;
        }
    }

    void runCycle()
    {
        float rawV = hal.readVoltage();
        float rawI = hal.readCurrent();
        float rawT = hal.readTempC();

        filteredV = iirFilter(filteredV, rawV);
        filteredI = iirFilter(filteredI, rawI);

        float rawLoadI = hal.readLoadCurrent();
        filteredLoadI = iirFilter(filteredLoadI, rawLoadI);

        filteredT = iirFilter(filteredT, rawT);

        compensatedAbsorptionV = getCompensatedVoltage(ABSORPTION_VOLTAGE);
        compensatedFloatV = getCompensatedVoltage(FLOAT_VOLTAGE);

        float vCal = applyCalOffset(filteredV, eeprom.vCalOffset);
        float iCal = applyCalOffset(filteredI, eeprom.iCalOffset);

        FaultCode trip = checkSafetyLimits(vCal, iCal, filteredT, state);
        if (trip != FaultCode::NONE) {
            if (state != ChargeState::FAULT) {
                faultReason = trip;
            }
            consecutiveFaults++;
            if (consecutiveFaults >= FAULT_DEBOUNCE_CYCLES) {
                ChargeState old = state;
                enterFault(faultReason != FaultCode::NONE ? faultReason : trip);
                if (old != ChargeState::FAULT) {
                    onStateTransition(old, state);
                }
            }
            if (hal.setPwm) {
                hal.setPwm(0);
            }
            if (hal.setChargeEnable) {
                hal.setChargeEnable(false);
            }
            return;
        }
        consecutiveFaults = 0;
        if (state != ChargeState::FAULT) {
            faultReason = FaultCode::NONE;
        }

        ChargeState old = state;
        switch (state) {
        case ChargeState::INIT:
            handleInit();
            break;
        case ChargeState::IDLE:
            handleIdle(vCal);
            break;
        case ChargeState::PRECHARGE:
            handlePrecharge(vCal, iCal);
            break;
        case ChargeState::BULK:
            handleBulk(vCal, iCal);
            break;
        case ChargeState::ABSORPTION:
            handleAbsorption(vCal, iCal);
            break;
        case ChargeState::FLOAT:
            handleFloat(vCal, iCal);
            break;
        case ChargeState::FAULT:
            handleFault();
            break;
        case ChargeState::RECOVERY:
            handleRecovery(vCal);
            break;
        }

        applyControlOutputs(vCal, iCal);

        accumulateCoulomb(eeprom, filteredI, filteredLoadI);

        if (hal.wakeCount && hal.lastEepromSave && (*hal.wakeCount - *hal.lastEepromSave) > EEPROM_SAVE_INTERVAL) {
            saveToEEPROM();
            *hal.lastEepromSave = *hal.wakeCount;
        }

        if (state != old) {
            onStateTransition(old, state);
            old = state;
        }

        updateTimers();

        if (state != old) {
            onStateTransition(old, state);
        }
    }

    void enterFault(FaultCode reason)
    {
        faultReason = reason;
        state = ChargeState::FAULT;
        absorptionMinutes = 0;
        prechargeTicks = 0;
        forceOutputsOff();
    }

    void forceOutputsOff()
    {
        if (hal.setPwm) {
            hal.setPwm(0);
        }
        if (hal.setChargeEnable) {
            hal.setChargeEnable(false);
        }
    }

    ChargeState getState() const { return state; }
    FaultCode getFault() const { return faultReason; }
    float getVoltage() const { return filteredV; }
    float getCurrent() const { return filteredI; }
    float getLoadCurrent() const { return filteredLoadI; }
    float getNetCurrent() const { return filteredI - filteredLoadI; }
    float getTemp() const { return filteredT; }
    float getTargetVoltage() const { return targetVoltage; }
    float getTargetCurrent() const { return targetCurrent; }
    float getCompensatedAbsorptionV() const { return compensatedAbsorptionV; }
    float getCompensatedFloatV() const { return compensatedFloatV; }
    uint8_t getPWM() const { return hal.getPwm ? hal.getPwm() : 0; }
    uint32_t getChargedAh() const { return milliAhToAh(eeprom.chargedMilliAmpHours); }
    uint32_t getDischargedAh() const { return milliAhToAh(eeprom.dischargedMilliAmpHours); }
    uint32_t getNetAh() const { return milliAhToAh(eeprom.chargedMilliAmpHours - eeprom.dischargedMilliAmpHours); }
    float getPwmIntegral() const { return pwmIntegral; }
    uint32_t getAbsorptionMinutes() const { return absorptionMinutes; }
    uint32_t getPrechargeTicks() const { return prechargeTicks; }
    uint32_t getFloatHours() const { return floatHours; }
    const EepromData &getEeprom() const { return eeprom; }
    uint8_t getConsecutiveFaults() const { return consecutiveFaults; }

    void requestResetFault()
    {
        if (state == ChargeState::FAULT) {
            faultReason = FaultCode::NONE;
            state = ChargeState::RECOVERY;
            absorptionMinutes = 0;
            prechargeTicks = 0;
        }
    }

    void adjustVoltageCal(float deltaVolts)
    {
        eeprom.vCalOffset = clampedCalTicks(eeprom.vCalOffset, deltaVolts, CAL_OFFSET_MAX_V);
    }
    void adjustCurrentCal(float deltaAmps)
    {
        eeprom.iCalOffset = clampedCalTicks(eeprom.iCalOffset, deltaAmps, CAL_OFFSET_MAX_A);
    }
    int16_t getVoltageCal() const { return eeprom.vCalOffset; }
    int16_t getCurrentCal() const { return eeprom.iCalOffset; }

    void setMasterEnable(bool en)
    {
        if (hal.masterEnable) {
            *hal.masterEnable = en;
        }
    }

    float getCompensatedVoltage(float baseVoltage) const { return compensateVoltage(baseVoltage, filteredT); }

  private:
    bool masterEnabled() const { return (hal.masterEnable == nullptr) || *hal.masterEnable; }

    static int16_t clampedCalTicks(int16_t ticks, float delta, float maxAbs)
    {
        if (!isfinite(delta)) {
            return ticks;
        }
        if (delta > maxAbs) {
            delta = maxAbs;
        }
        if (delta < -maxAbs) {
            delta = -maxAbs;
        }
        const int16_t maxTicks = roundToCalTicks(maxAbs);
        int32_t next = (int32_t)ticks + (int32_t)roundToCalTicks(delta);
        return clampI16(static_cast<int16_t>(next), static_cast<int16_t>(-maxTicks), maxTicks);
    }

    void handleInit()
    {
        if (filteredV < 3.0f) {
            enterFault(FaultCode::UNDER_VOLTAGE);
            return;
        }
        state = ChargeState::IDLE;
    }

    void handleIdle(float v)
    {
        targetCurrent = 0.0f;
        targetVoltage = compensatedFloatV;
        state = nextIdleState(v, filteredT, masterEnabled());
    }

    void handlePrecharge(float v, float i)
    {
        (void)i;
        targetCurrent = PRECHARGE_CURRENT;
        targetVoltage = compensatedAbsorptionV;
        if (prechargeExitToBulk(v)) {
            state = ChargeState::BULK;
        }
        if (prechargeTimedOut(prechargeTicks)) {
            enterFault(FaultCode::CHARGE_TIMEOUT);
        }
    }

    void handleBulk(float v, float i)
    {
        targetCurrent = MAX_CHARGE_CURRENT;
        targetVoltage = compensatedAbsorptionV;
        if (bulkEnterAbsorption(v, i, compensatedAbsorptionV)) {
            absorptionMinutes = 0;
            state = ChargeState::ABSORPTION;
        }
    }

    void handleAbsorption(float v, float i)
    {
        (void)v;
        targetCurrent = MAX_CHARGE_CURRENT * 0.6f;
        targetVoltage = compensatedAbsorptionV;
        if (absorptionEnterFloat(i, absorptionMinutes)) {
            floatHours = 0;
            state = ChargeState::FLOAT;
        }
    }

    void handleFloat(float v, float i)
    {
        (void)i;
        targetCurrent = MAX_CHARGE_CURRENT * 0.15f;
        targetVoltage = compensatedFloatV;
        if (floatRestartBulk(v)) {
            state = ChargeState::BULK;
        }
    }

    void handleFault()
    {
        targetCurrent = 0.0f;
        targetVoltage = 0.0f;
        forceOutputsOff();
    }

    void handleRecovery(float v)
    {
        (void)v;
        state = ChargeState::IDLE;
    }

    void applyControlOutputs(float v, float i)
    {
        uint8_t dutyNow = hal.getPwm ? hal.getPwm() : 0;
        ControlLawOutput out =
          applyControlLaw(state, masterEnabled(), v, i, targetVoltage, targetCurrent, dutyNow, pwmIntegral);
        pwmIntegral = out.pwmIntegral;
        if (hal.setPwm) {
            hal.setPwm(out.duty);
        }
        if (hal.setChargeEnable) {
            hal.setChargeEnable(out.enable);
        }
    }

    void updateTimers()
    {
        if (state == ChargeState::ABSORPTION) {
            absorptionMinutes++;
        }
        if (state == ChargeState::PRECHARGE) {
            prechargeTicks++;
        }
        if (state == ChargeState::FLOAT) {
            floatHours++;
            if (floatHours >= floatRechargeWakeThreshold()) {
                floatHours = 0;
                state = ChargeState::BULK;
            }
        }
    }

    void onStateTransition(ChargeState from, ChargeState to)
    {
        pwmIntegral = 0.0f;
        if (to == ChargeState::PRECHARGE) {
            prechargeTicks = 0;
        }
        if (to == ChargeState::BULK || to == ChargeState::ABSORPTION) {
            if (hal.setChargeEnable) {
                hal.setChargeEnable(true);
            }
        }
        if (to == ChargeState::FAULT) {
            forceOutputsOff();
        }

        eeprom.lastState = static_cast<uint8_t>(to);
        if (isCriticalTransition(from, to)) {
            saveToEEPROM();
            if (hal.lastEepromSave && hal.wakeCount) {
                *hal.lastEepromSave = *hal.wakeCount;
            }
        }
    }

    void loadFromEEPROM() { persistenceLoad(eeprom, hal.eepromLoad, hal.eepromSave); }

    void saveToEEPROM() { persistenceSave(eeprom, hal.eepromSave); }

    ChargerHal hal;
    ChargeState state;
    float filteredV;
    float filteredI;
    float filteredLoadI;
    float filteredT;
    float targetCurrent;
    float targetVoltage;
    float compensatedAbsorptionV;
    float compensatedFloatV;
    float pwmIntegral;
    uint32_t absorptionMinutes;
    uint32_t prechargeTicks;
    uint32_t floatHours;
    FaultCode faultReason;
    uint8_t consecutiveFaults;
    EepromData eeprom;
};

#endif // BATTERY_MANAGER_CHARGER_CONTROLLER_H
