#include "Config.h"
#include "coulomb.h"
#include "mock_hal.h"
#include "sensor_math.h"
#include "unity.h"

void test_sensor_voltage_full_scale(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.0f * VOLTAGE_DIVIDER_RATIO, adcToVoltage(1023.0f));
}

void test_sensor_voltage_zero(void) { TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, adcToVoltage(0.0f)); }

void test_sensor_voltage_mid_scale(void)
{
    const float raw = 512.0f;
    const float expected = (raw / 1023.0f) * ADC_REFERENCE_V * VOLTAGE_DIVIDER_RATIO;
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, adcToVoltage(raw));
}

void test_sensor_current_zero_point(void) { TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, adcToCurrent(CURRENT_ZERO_POINT)); }

void test_sensor_current_positive_above_zero(void)
{
    float amps = adcToCurrent(CURRENT_ZERO_POINT + 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f * CURRENT_SCALE, amps);
    TEST_ASSERT_TRUE(amps > 0.0f);
}

void test_sensor_current_below_zero_clamped(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, adcToCurrent(CURRENT_ZERO_POINT - 50.0f));
}

void test_sensor_load_current_same_scale(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 20.0f * CURRENT_SCALE, adcToCurrent(CURRENT_ZERO_POINT + 20.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, adcToCurrent(CURRENT_ZERO_POINT));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, adcToCurrent(0.0f));
}

void test_sensor_load_discharges_controller(void)
{
    MockHal mock;
    mock.voltage = RECHARGE_VOLTAGE + 0.3f;
    mock.current = 0.0f;
    mock.loadCurrent = 1.0f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    settle(c, mock, 30);
    const uint32_t before = c.getEeprom().dischargedMilliAmpHours;
    const uint32_t n = 50;
    runCycles(c, mock, n);
    const float mAh = 1.0f * (SLEEP_INTERVAL_MS / 3600000.0f);
    const uint32_t per = (uint32_t)(mAh * 1000.0f + 0.5f);
    TEST_ASSERT_EQUAL_UINT32(before + n * per, c.getEeprom().dischargedMilliAmpHours);
    TEST_ASSERT_EQUAL_UINT32(0, c.getEeprom().chargedMilliAmpHours);
}

void test_sensor_iir_on_controller(void)
{
    MockHal mock;
    const float v0 = RECHARGE_VOLTAGE + 0.3f;
    mock.voltage = v0;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    runCycles(c, mock, 8);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, v0, c.getVoltage());
    const float delta = 1.0f;
    mock.voltage = v0 + delta;
    runCycles(c, mock, 1);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, v0 + 0.25f * delta, c.getVoltage());
    settle(c, mock, 24);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, v0 + delta, c.getVoltage());
}

void test_sensor_ntc_25c_half_bridge(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 25.0f, ntcRawToTempC(511.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 25.0f, ntcRawToTempC(512.0f));
}

void test_sensor_ntc_open_short(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 99.9f, ntcRawToTempC(1023.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 99.9f, ntcRawToTempC(0.0f));
    const float rawNear5 = ((ADC_REFERENCE_V - 0.01f) / ADC_REFERENCE_V) * 1023.0f;
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 99.9f, ntcRawToTempC(rawNear5));
    const float rawNear0 = (0.01f / ADC_REFERENCE_V) * 1023.0f;
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 99.9f, ntcRawToTempC(rawNear0));
}

void test_sensor_iir_alpha_025(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, iirFilter(0.0f, 4.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.75f * 1.0f + 0.25f * 4.0f, iirFilter(1.0f, 4.0f));
}

void test_sensor_cal_offset(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 13.12f, applyCalOffset(13.00f, 12));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 12.99f, applyCalOffset(13.00f, -1));
}

void run_sensor_math_tests(void)
{
    RUN_TEST(test_sensor_voltage_full_scale);
    RUN_TEST(test_sensor_voltage_zero);
    RUN_TEST(test_sensor_voltage_mid_scale);
    RUN_TEST(test_sensor_current_zero_point);
    RUN_TEST(test_sensor_current_positive_above_zero);
    RUN_TEST(test_sensor_current_below_zero_clamped);
    RUN_TEST(test_sensor_load_current_same_scale);
    RUN_TEST(test_sensor_load_discharges_controller);
    RUN_TEST(test_sensor_iir_on_controller);
    RUN_TEST(test_sensor_ntc_25c_half_bridge);
    RUN_TEST(test_sensor_ntc_open_short);
    RUN_TEST(test_sensor_iir_alpha_025);
    RUN_TEST(test_sensor_cal_offset);
}
