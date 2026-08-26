#include "Config.h"
#include "mock_hal.h"
#include "safety.h"
#include "unity.h"

static int asInt(FaultCode f) { return static_cast<int>(f); }

static int asInt(ChargeState s) { return static_cast<int>(s); }

void test_safety_over_voltage(void)
{
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::OVER_VOLTAGE),
      asInt(checkSafetyLimits(MAX_CHARGE_VOLTAGE + 0.01f, 0.0f, 25.0f, ChargeState::IDLE)));
}

void test_safety_under_voltage_not_init(void)
{
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::UNDER_VOLTAGE),
      asInt(checkSafetyLimits(MIN_OPERATING_VOLTAGE - 0.01f, 0.0f, 25.0f, ChargeState::IDLE)));
}

void test_safety_under_voltage_skipped_in_init(void)
{
    TEST_ASSERT_EQUAL_INT(
      asInt(FaultCode::NONE), asInt(checkSafetyLimits(MIN_OPERATING_VOLTAGE - 0.01f, 0.0f, 25.0f, ChargeState::INIT)));
}

void test_safety_over_current(void)
{
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::OVER_CURRENT),
      asInt(checkSafetyLimits(NOMINAL_VOLTAGE, MAX_CHARGE_CURRENT * 1.25f + 0.01f, 25.0f, ChargeState::BULK)));
}

void test_safety_over_temp(void)
{
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::OVER_TEMP),
      asInt(checkSafetyLimits(NOMINAL_VOLTAGE, 0.0f, MAX_TEMP_C + 0.01f, ChargeState::IDLE)));
}

void test_safety_under_temp_in_bulk_or_absorption(void)
{
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::UNDER_TEMP),
      asInt(checkSafetyLimits(NOMINAL_VOLTAGE, 0.0f, MIN_TEMP_C - 0.01f, ChargeState::BULK)));
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::UNDER_TEMP),
      asInt(checkSafetyLimits(NOMINAL_VOLTAGE, 0.0f, MIN_TEMP_C - 0.01f, ChargeState::ABSORPTION)));
}

void test_safety_under_temp_skipped_in_idle(void)
{
    TEST_ASSERT_EQUAL_INT(
      asInt(FaultCode::NONE), asInt(checkSafetyLimits(NOMINAL_VOLTAGE, 0.0f, MIN_TEMP_C - 0.01f, ChargeState::IDLE)));
}

void test_safety_nominal_none(void)
{
    TEST_ASSERT_EQUAL_INT(
      asInt(FaultCode::NONE), asInt(checkSafetyLimits(NOMINAL_VOLTAGE, 0.0f, 25.0f, ChargeState::IDLE)));
}

void test_safety_never_sensor_or_manual(void)
{
    TEST_ASSERT_NOT_EQUAL(
      asInt(FaultCode::SENSOR_FAULT), asInt(checkSafetyLimits(0.0f, 100.0f, 99.9f, ChargeState::BULK)));
    TEST_ASSERT_NOT_EQUAL(
      asInt(FaultCode::MANUAL), asInt(checkSafetyLimits(MAX_CHARGE_VOLTAGE + 1.0f, 0.0f, 25.0f, ChargeState::IDLE)));
}

void test_safety_debounce_and_latch(void)
{
    MockHal mock;
    mock.voltage = MAX_CHARGE_VOLTAGE + 0.05f;
    mock.tempC = 25.0f;
    mock.current = 0.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::IDLE), asInt(c.getState()));

    runCycles(c, mock, FAULT_DEBOUNCE_CYCLES - 1);
    TEST_ASSERT_NOT_EQUAL(asInt(ChargeState::FAULT), asInt(c.getState()));
    TEST_ASSERT_EQUAL_UINT8(0, c.getPWM());
    TEST_ASSERT_FALSE(mock.chargeEnable);

    mock.voltage = RECHARGE_VOLTAGE + 0.3f;
    runCycles(c, mock, 1);
    TEST_ASSERT_EQUAL_UINT8(0, c.getConsecutiveFaults());
    TEST_ASSERT_NOT_EQUAL(asInt(ChargeState::FAULT), asInt(c.getState()));

    mock.voltage = MAX_CHARGE_VOLTAGE + 0.05f;
    runCycles(c, mock, 20);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(c.getState()));
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::OVER_VOLTAGE), asInt(c.getFault()));
    TEST_ASSERT_EQUAL_UINT8(0, c.getPWM());
    TEST_ASSERT_FALSE(mock.chargeEnable);

    ChargeState frozen = c.getState();
    mock.voltage = RECHARGE_VOLTAGE + 0.3f;
    runCycles(c, mock, 8);
    TEST_ASSERT_EQUAL_INT(asInt(frozen), asInt(c.getState()));
}

#if defined(BATTERY_PROFILE_LIFEPO4_4S)
void test_safety_lifepo4_under_temp_at_min(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, MIN_TEMP_C);
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::UNDER_TEMP),
      asInt(checkSafetyLimits(NOMINAL_VOLTAGE, 0.0f, MIN_TEMP_C - 0.01f, ChargeState::BULK)));
}
#endif

#if defined(BATTERY_PROFILE_LIION_4S)
void test_safety_liion_over_current_at_125pct(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f, MAX_CHARGE_CURRENT);
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::OVER_CURRENT),
      asInt(checkSafetyLimits(NOMINAL_VOLTAGE, MAX_CHARGE_CURRENT * 1.25f + 0.01f, 25.0f, ChargeState::BULK)));
    TEST_ASSERT_EQUAL_INT(asInt(FaultCode::NONE),
      asInt(checkSafetyLimits(NOMINAL_VOLTAGE, MAX_CHARGE_CURRENT * 1.25f, 25.0f, ChargeState::BULK)));
}
#endif

void run_safety_tests(void)
{
    RUN_TEST(test_safety_over_voltage);
    RUN_TEST(test_safety_under_voltage_not_init);
    RUN_TEST(test_safety_under_voltage_skipped_in_init);
    RUN_TEST(test_safety_over_current);
    RUN_TEST(test_safety_over_temp);
    RUN_TEST(test_safety_under_temp_in_bulk_or_absorption);
    RUN_TEST(test_safety_under_temp_skipped_in_idle);
    RUN_TEST(test_safety_nominal_none);
    RUN_TEST(test_safety_never_sensor_or_manual);
    RUN_TEST(test_safety_debounce_and_latch);
#if defined(BATTERY_PROFILE_LIFEPO4_4S)
    RUN_TEST(test_safety_lifepo4_under_temp_at_min);
#endif
#if defined(BATTERY_PROFILE_LIION_4S)
    RUN_TEST(test_safety_liion_over_current_at_125pct);
#endif
}
