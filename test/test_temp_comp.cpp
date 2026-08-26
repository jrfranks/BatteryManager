#include "Config.h"
#include "temp_comp.h"
#include "unity.h"

void test_temp_comp_identity_at_25c(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, ABSORPTION_VOLTAGE, compensateVoltage(ABSORPTION_VOLTAGE, 25.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, FLOAT_VOLTAGE, compensateVoltage(FLOAT_VOLTAGE, 25.0f));
}

void test_temp_comp_hot_lowers_voltage(void)
{
    const float expected = ABSORPTION_VOLTAGE + (TEMP_COMP_mV_PER_C / 1000.0f) * (35.0f - 25.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, compensateVoltage(ABSORPTION_VOLTAGE, 35.0f));
#if defined(BATTERY_PROFILE_LEAD_ACID_12V)
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 14.15f, compensateVoltage(14.40f, 35.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.25f, ABSORPTION_VOLTAGE - expected);
#endif
}

void test_temp_comp_cold_raises_voltage(void)
{
    const float expected = ABSORPTION_VOLTAGE + (TEMP_COMP_mV_PER_C / 1000.0f) * (15.0f - 25.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, compensateVoltage(ABSORPTION_VOLTAGE, 15.0f));
#if defined(BATTERY_PROFILE_LEAD_ACID_12V)
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 14.65f, compensateVoltage(14.40f, 15.0f));
#endif
}

void test_temp_comp_k_zero_identity(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 14.40f, compensateVoltage(14.40f, 40.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 13.50f, compensateVoltage(13.50f, 0.0f, 0.0f));
}

void test_temp_comp_float_and_absorption_same_delta(void)
{
    const float dAbs = compensateVoltage(ABSORPTION_VOLTAGE, 35.0f) - ABSORPTION_VOLTAGE;
    const float dFloat = compensateVoltage(FLOAT_VOLTAGE, 35.0f) - FLOAT_VOLTAGE;
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, dAbs, dFloat);
}

void test_temp_comp_clamped_to_max(void)
{
    TEST_ASSERT_TRUE(compensateVoltage(ABSORPTION_VOLTAGE, MIN_TEMP_C) <= MAX_CHARGE_VOLTAGE + 1e-4f);
}

void run_temp_comp_tests(void)
{
    RUN_TEST(test_temp_comp_identity_at_25c);
    RUN_TEST(test_temp_comp_hot_lowers_voltage);
    RUN_TEST(test_temp_comp_cold_raises_voltage);
    RUN_TEST(test_temp_comp_k_zero_identity);
    RUN_TEST(test_temp_comp_float_and_absorption_same_delta);
    RUN_TEST(test_temp_comp_clamped_to_max);
}
