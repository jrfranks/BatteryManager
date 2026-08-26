#include "mock_hal.h"
#include "telemetry.h"
#include "unity.h"
#include <stdio.h>
#include <string.h>

void test_telemetry_csv_fields(void)
{
    MockHal mock;
    mock.voltage = NOMINAL_VOLTAGE;
    mock.tempC = 25.0f;
    mock.current = 0.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    runCycles(c, mock, 2);

    char buf[192];
    formatTelemetryCsv(buf, sizeof(buf), c);
    TEST_ASSERT_EQUAL_STRING_LEN("BM,", buf, 3);

    int state = 0, pwm = 0, fault = 0;
    unsigned charged = 0, discharged = 0;
    float v = 0, i = 0, load = 0, net = 0, t = 0;
    int n = sscanf(
      buf, "BM,%d,%f,%f,%f,%f,%f,%d,%d,%u,%u", &state, &v, &i, &load, &net, &t, &pwm, &fault, &charged, &discharged);
    TEST_ASSERT_EQUAL_INT(10, n);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(c.getState()), state);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, c.getVoltage(), v);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(c.getPWM()), pwm);
}

void test_telemetry_json_keys(void)
{
    MockHal mock;
    mock.voltage = NOMINAL_VOLTAGE;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    runCycles(c, mock, 2);

    char buf[384];
    formatTelemetryJson(buf, sizeof(buf), c);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"state\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"v\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"i_charge\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"i_load\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"i_net\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"temp\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"pwm\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"target_v\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"comp_absorb_v\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"comp_float_v\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"charged_ah\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"discharged_ah\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"net_ah\""));
}

void test_telemetry_null_sink_safe(void)
{
    MockHal mock;
    mock.voltage = NOMINAL_VOLTAGE;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    formatTelemetryCsv(nullptr, 80, c);
    formatTelemetryJson(nullptr, 80, c);
    char empty[1];
    formatTelemetryCsv(empty, 0, c);
    formatTelemetryJson(empty, 0, c);
}

void run_telemetry_tests(void)
{
    RUN_TEST(test_telemetry_csv_fields);
    RUN_TEST(test_telemetry_json_keys);
    RUN_TEST(test_telemetry_null_sink_safe);
}
