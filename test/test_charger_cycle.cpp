#include "Config.h"
#include "mock_hal.h"
#include "serial_parse.h"
#include "unity.h"

static int asInt(ChargeState s) { return static_cast<int>(s); }
static int asInt(FaultCode f) { return static_cast<int>(f); }

void test_cycle_boot_init_idle(void)
{
    MockHal mock;
    mock.voltage = RECHARGE_VOLTAGE + 0.3f;
    mock.tempC = 25.0f;
    mock.current = 0.0f;
    ChargerController c(mock.hal());
    c.begin();
    runCycles(c, mock, 2);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(c.getState()));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, RECHARGE_VOLTAGE + 0.3f, c.getVoltage());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 25.0f, c.getTemp());
}

void test_cycle_drop_to_bulk_enable_high(void)
{
    MockHal mock;
    mock.voltage = RECHARGE_VOLTAGE + 0.3f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    runCycles(c, mock, 2);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(c.getState()));

    mock.voltage = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    settle(c, mock, 24);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
    bool sawEnable = mock.chargeEnable;
    for (int n = 0; n < 20 && !sawEnable; ++n) {
        runCycles(c, mock, 1);
        sawEnable = mock.chargeEnable || c.getPWM() > 5;
    }
    TEST_ASSERT_TRUE(sawEnable);
}

void test_cycle_bulk_absorb_float(void)
{
    MockHal mock;
    mock.voltage = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    mock.current = MAX_CHARGE_CURRENT * 0.5f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::BULK);
    ChargerController c(mock.hal());
    c.begin();
    settle(c, mock, 8);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));

    mock.voltage = ABSORPTION_VOLTAGE;
    mock.current = MAX_CHARGE_CURRENT * 0.5f;
    settle(c, mock, 30);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::ABSORPTION), asInt(c.getState()));

    mock.current = ABSORPTION_EXIT_CURRENT * 0.5f;
    settle(c, mock, 24);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FLOAT), asInt(c.getState()));
}

void test_cycle_overvoltage_latch_and_reset(void)
{
    MockHal mock;
    mock.voltage = NOMINAL_VOLTAGE;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    runCycles(c, mock, 2);

    mock.voltage = MAX_CHARGE_VOLTAGE + 2.0f;
    runCycles(c, mock, 30);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(c.getState()));
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::OVER_VOLTAGE), asInt(c.getFault()));
    TEST_ASSERT_FALSE(mock.chargeEnable);
    TEST_ASSERT_EQUAL_UINT8(0, c.getPWM());

    mock.voltage = RECHARGE_VOLTAGE + 0.3f;
    settle(c, mock, 24);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(c.getState()));
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::OVER_VOLTAGE), asInt(c.getFault()));
    c.requestResetFault();
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::RECOVERY), asInt(c.getState()));
    runCycles(c, mock, 1);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(c.getState()));
}

void test_cycle_deep_discharge_precharge(void)
{
    MockHal mock;
    mock.voltage = PRECHARGE_VOLTAGE - 1.0f;
    if (mock.voltage < MIN_OPERATING_VOLTAGE) {
        mock.voltage = (MIN_OPERATING_VOLTAGE + PRECHARGE_VOLTAGE) * 0.5f;
    }
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    settle(c, mock, 8);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::PRECHARGE), asInt(c.getState()));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, PRECHARGE_CURRENT, c.getTargetCurrent());
}

void test_cycle_calibration_applies_to_safety_and_fsm(void)
{
    MockHal mock;
    mock.voltage = MAX_CHARGE_VOLTAGE - 0.4f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE, 0, 0, 0, 0);
    ChargerController c(mock.hal());
    c.begin();
    c.adjustVoltageCal(CAL_OFFSET_MAX_V);
    mock.voltage = MAX_CHARGE_VOLTAGE - 0.4f;
    runCycles(c, mock, FAULT_DEBOUNCE_CYCLES + 2);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(c.getState()));
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::OVER_VOLTAGE), asInt(c.getFault()));
}

void test_cycle_coulomb_net_only(void)
{
    MockHal mock;
    mock.voltage = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    mock.current = 2.0f;
    mock.loadCurrent = 0.5f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::BULK);
    ChargerController c(mock.hal());
    c.begin();
    const uint32_t n = 50;
    runCycles(c, mock, n);
    TEST_ASSERT_TRUE(c.getEeprom().chargedMilliAmpHours > 0);
    TEST_ASSERT_EQUAL_UINT32(0, c.getEeprom().dischargedMilliAmpHours);
}

void test_cycle_stop_forces_pwm_zero_in_bulk(void)
{
    MockHal mock;
    mock.voltage = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    mock.current = 0.0f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::BULK);
    ChargerController c(mock.hal());
    c.begin();
    runCycles(c, mock, 16);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
    TEST_ASSERT_TRUE(c.getPWM() > 0 || mock.chargeEnable);

    executeCommand(parseCommandLine("stop"), c);
    TEST_ASSERT_EQUAL_UINT8(0, c.getPWM());
    TEST_ASSERT_FALSE(mock.chargeEnable);
    runCycles(c, mock, 1);
    TEST_ASSERT_EQUAL_UINT8(0, c.getPWM());
    TEST_ASSERT_FALSE(mock.chargeEnable);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
}

void test_cycle_temp_comp_hot_lowers_absorption(void)
{
    MockHal mock;
    mock.voltage = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    mock.current = MAX_CHARGE_CURRENT * 0.5f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::BULK);
    ChargerController c(mock.hal());
    c.begin();
    settle(c, mock, 24);
    const float at25 = c.getCompensatedAbsorptionV();
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, ABSORPTION_VOLTAGE, at25);

    mock.tempC = 35.0f;
    settle(c, mock, 24);
    const float expected = ABSORPTION_VOLTAGE + (TEMP_COMP_mV_PER_C / 1000.0f) * (35.0f - 25.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, expected, c.getCompensatedAbsorptionV());
#if defined(BATTERY_PROFILE_LEAD_ACID_12V)
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 14.15f, c.getCompensatedAbsorptionV());
#endif
}

void run_charger_cycle_tests(void)
{
    RUN_TEST(test_cycle_boot_init_idle);
    RUN_TEST(test_cycle_drop_to_bulk_enable_high);
    RUN_TEST(test_cycle_bulk_absorb_float);
    RUN_TEST(test_cycle_overvoltage_latch_and_reset);
    RUN_TEST(test_cycle_deep_discharge_precharge);
    RUN_TEST(test_cycle_calibration_applies_to_safety_and_fsm);
    RUN_TEST(test_cycle_coulomb_net_only);
    RUN_TEST(test_cycle_stop_forces_pwm_zero_in_bulk);
    RUN_TEST(test_cycle_temp_comp_hot_lowers_absorption);
}
