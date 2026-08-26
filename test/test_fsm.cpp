#include "Config.h"
#include "charge_fsm.h"
#include "mock_hal.h"
#include "unity.h"

static int asInt(ChargeState s) { return static_cast<int>(s); }
static int asInt(FaultCode f) { return static_cast<int>(f); }

static ChargerController bootAt(MockHal &mock, ChargeState last, float v, float t = 25.0f, float i = 0.0f)
{
    mock.voltage = v;
    mock.tempC = t;
    mock.current = i;
    plantValidEeprom(mock, last);
    ChargerController c(mock.hal());
    c.begin();
    return c;
}

void test_fsm_init_undervoltage(void)
{
    MockHal mock;
    ChargerController c = bootAt(mock, ChargeState::INIT, 2.5f);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::INIT), asInt(c.getState()));
    runCycles(c, mock, 1);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(c.getState()));
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::UNDER_VOLTAGE), asInt(c.getFault()));
}

void test_fsm_init_to_idle(void)
{
    MockHal mock;
    ChargerController c = bootAt(mock, ChargeState::INIT, RECHARGE_VOLTAGE + 0.3f);
    runCycles(c, mock, 1);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(c.getState()));
}

void test_fsm_idle_master_disabled(void)
{
    MockHal mock;
    mock.masterEnable = false;
    ChargerController c = bootAt(mock, ChargeState::IDLE, PRECHARGE_VOLTAGE - 0.5f);
    settle(c, mock);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(c.getState()));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.getTargetCurrent());
}

void test_fsm_idle_temp_out_of_range(void)
{
    MockHal mock;
    ChargerController c = bootAt(mock, ChargeState::IDLE, PRECHARGE_VOLTAGE - 0.5f, MIN_TEMP_C - 5.0f);
    settle(c, mock);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(c.getState()));

    const float vBulk = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(nextIdleState(vBulk, MAX_TEMP_C + 1.0f, true)));
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(nextIdleState(vBulk, MIN_TEMP_C - 1.0f, true)));
}

void test_fsm_idle_to_precharge(void)
{
    MockHal mock;
    ChargerController c = bootAt(mock, ChargeState::IDLE, PRECHARGE_VOLTAGE - 0.2f);
    settle(c, mock);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::PRECHARGE), asInt(c.getState()));
}

void test_fsm_idle_to_bulk(void)
{
    MockHal mock;
    const float v = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    ChargerController c = bootAt(mock, ChargeState::IDLE, v);
    settle(c, mock);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
}

void test_fsm_idle_stay_when_high(void)
{
    MockHal mock;
    ChargerController c = bootAt(mock, ChargeState::IDLE, RECHARGE_VOLTAGE + 0.2f);
    settle(c, mock);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(c.getState()));
}

void test_fsm_precharge_to_bulk(void)
{
    MockHal mock;
    ChargerController c = bootAt(mock, ChargeState::PRECHARGE, PRECHARGE_VOLTAGE + 0.4f);
    settle(c, mock);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
}

void test_fsm_precharge_timeout(void)
{
    MockHal mock;
    float v = PRECHARGE_VOLTAGE - 0.2f;
    if (v < MIN_OPERATING_VOLTAGE) {
        v = (MIN_OPERATING_VOLTAGE + PRECHARGE_VOLTAGE) * 0.5f;
    }
    ChargerController c = bootAt(mock, ChargeState::PRECHARGE, v);
    runCycles(c, mock, PRECHARGE_MAX_MINUTES + 2);
    TEST_ASSERT_NOT_EQUAL(asInt(ChargeState::FAULT), asInt(c.getState()));
    runCycles(c, mock, prechargeTimeoutWakes() + 2);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(c.getState()));
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::CHARGE_TIMEOUT), asInt(c.getFault()));

    settle(c, mock, 24);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(c.getState()));
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::CHARGE_TIMEOUT), asInt(c.getFault()));

    c.requestResetFault();
    settle(c, mock, 8);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::PRECHARGE), asInt(c.getState()));
    runCycles(c, mock, PRECHARGE_MAX_MINUTES + 2);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::PRECHARGE), asInt(c.getState()));
}

void test_fsm_bulk_to_absorption(void)
{
    MockHal mock;
    const float v = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    ChargerController c = bootAt(mock, ChargeState::BULK, v, 25.0f, MAX_CHARGE_CURRENT);
    settle(c, mock, 8);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, MAX_CHARGE_CURRENT, c.getTargetCurrent());

    mock.voltage = ABSORPTION_VOLTAGE;
    mock.current = MAX_CHARGE_CURRENT * 0.5f;
    uint32_t guard = 0;
    while (c.getState() != ChargeState::ABSORPTION && guard++ < 40) {
        runCycles(c, mock, 1);
    }
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::ABSORPTION), asInt(c.getState()));
    TEST_ASSERT_TRUE(c.getAbsorptionMinutes() <= 2);
}

void test_fsm_bulk_stays_otherwise(void)
{
    MockHal mock;
    const float v = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    ChargerController c = bootAt(mock, ChargeState::BULK, v, 25.0f, MAX_CHARGE_CURRENT);
    settle(c, mock, 8);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, MAX_CHARGE_CURRENT, c.getTargetCurrent());
}

void test_fsm_absorption_to_float_on_current(void)
{
    MockHal mock;
    const float vAbs = MAX_CHARGE_VOLTAGE - 0.4f;
    ChargerController c = bootAt(mock, ChargeState::ABSORPTION, vAbs, 25.0f, ABSORPTION_EXIT_CURRENT - 0.05f);
    uint32_t guard = 0;
    while (c.getState() != ChargeState::FLOAT && guard++ < 16) {
        runCycles(c, mock, 1);
    }
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FLOAT), asInt(c.getState()));
    TEST_ASSERT_TRUE(c.getFloatHours() <= 2);
}

void test_fsm_absorption_to_float_on_minutes(void)
{
    MockHal mock;
    const float vAbs = MAX_CHARGE_VOLTAGE - 0.4f;
    ChargerController c = bootAt(mock, ChargeState::ABSORPTION, vAbs, 25.0f, MAX_CHARGE_CURRENT * 0.5f);
    runCycles(c, mock, ABSORPTION_MAX_MINUTES + 2);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::ABSORPTION), asInt(c.getState()));
    runCycles(c, mock, absorptionTimeoutWakes() + 2);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FLOAT), asInt(c.getState()));
}

void test_fsm_float_to_bulk_on_voltage(void)
{
    MockHal mock;
    ChargerController c = bootAt(mock, ChargeState::FLOAT, RECHARGE_VOLTAGE + 0.3f);
    settle(c, mock, 6);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FLOAT), asInt(c.getState()));
    mock.voltage = RECHARGE_VOLTAGE - 0.2f;
    settle(c, mock, 24);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
}

void test_fsm_float_forced_recharge(void)
{
    MockHal mock;
    ChargerController c = bootAt(mock, ChargeState::FLOAT, RECHARGE_VOLTAGE + 0.4f);
    const uint64_t need = floatRechargeWakeThreshold();
    uint32_t n = 0;
    while (c.getState() != ChargeState::BULK && n++ < static_cast<uint32_t>(need) + 2) {
        runCycles(c, mock, 1);
    }
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ChargeState::BULK), c.getEeprom().lastState);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ChargeState::BULK), mock.peekEeprom().lastState);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, c.getPwmIntegral());
}

void test_fsm_fault_until_reset_then_recovery_idle(void)
{
    MockHal mock;
    ChargerController c = bootAt(mock, ChargeState::INIT, 2.0f);
    runCycles(c, mock, 1);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(c.getState()));
    mock.voltage = NOMINAL_VOLTAGE;
    runCycles(c, mock, 4);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(c.getState()));
    c.requestResetFault();
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::RECOVERY), asInt(c.getState()));
    runCycles(c, mock, 1);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(c.getState()));
}

void test_fsm_begin_restore_and_force_idle(void)
{
    {
        MockHal mock;
        ChargerController ok = bootAt(mock, ChargeState::BULK, (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f);
        TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(ok.getState()));
    }
    {
        MockHal mockInv;
        ChargerController inv = bootAt(mockInv, ChargeState::FAULT, NOMINAL_VOLTAGE);
        TEST_ASSERT_EQUAL_INT(asInt(ChargeState::INIT), asInt(inv.getState()));
    }
    {
        MockHal mockHigh;
        ChargerController forced = bootAt(mockHigh, ChargeState::BULK, MAX_CHARGE_VOLTAGE - 0.1f);
        TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(forced.getState()));
    }
}

#if defined(BATTERY_PROFILE_LIION_4S)
void test_fsm_liion_restore_absorption(void)
{
    MockHal mock;
    ChargerController c = bootAt(mock, ChargeState::ABSORPTION, ABSORPTION_VOLTAGE);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::ABSORPTION), asInt(c.getState()));
}
#endif

void test_fsm_recovery_always_idle(void)
{
    MockHal mock;
    mock.voltage = 2.0f;
    plantValidEeprom(mock, ChargeState::INIT);
    ChargerController c(mock.hal());
    c.begin();
    runCycles(c, mock, 1);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(c.getState()));
    mock.voltage = RECHARGE_VOLTAGE + 0.3f;
    settle(c, mock, 24);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(c.getState()));
    c.requestResetFault();
    runCycles(c, mock, 1);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(c.getState()));
}

void run_fsm_tests(void)
{
    RUN_TEST(test_fsm_init_undervoltage);
    RUN_TEST(test_fsm_init_to_idle);
    RUN_TEST(test_fsm_idle_master_disabled);
    RUN_TEST(test_fsm_idle_temp_out_of_range);
    RUN_TEST(test_fsm_idle_to_precharge);
    RUN_TEST(test_fsm_idle_to_bulk);
    RUN_TEST(test_fsm_idle_stay_when_high);
    RUN_TEST(test_fsm_precharge_to_bulk);
    RUN_TEST(test_fsm_precharge_timeout);
    RUN_TEST(test_fsm_bulk_to_absorption);
    RUN_TEST(test_fsm_bulk_stays_otherwise);
    RUN_TEST(test_fsm_absorption_to_float_on_current);
    RUN_TEST(test_fsm_absorption_to_float_on_minutes);
    RUN_TEST(test_fsm_float_to_bulk_on_voltage);
    RUN_TEST(test_fsm_float_forced_recharge);
    RUN_TEST(test_fsm_fault_until_reset_then_recovery_idle);
    RUN_TEST(test_fsm_begin_restore_and_force_idle);
#if defined(BATTERY_PROFILE_LIION_4S)
    RUN_TEST(test_fsm_liion_restore_absorption);
#endif
    RUN_TEST(test_fsm_recovery_always_idle);
}
