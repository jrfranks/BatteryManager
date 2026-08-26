#include "Config.h"
#include "coulomb.h"
#include "mock_hal.h"
#include "unity.h"

void test_coulomb_deadband_no_accumulate(void)
{
    EepromData e{};
    eepromInitDefaults(e);
    accumulateCoulomb(e, 0.02f, 0.0f);
    accumulateCoulomb(e, 0.0f, 0.02f);
    accumulateCoulomb(e, 0.01f, 0.0f);
    accumulateCoulomb(e, 0.0f, 0.01f);
    TEST_ASSERT_EQUAL_UINT32(0, e.chargedMilliAmpHours);
    TEST_ASSERT_EQUAL_UINT32(0, e.dischargedMilliAmpHours);
}

void test_coulomb_one_amp_1800_wakes(void)
{
    EepromData e{};
    eepromInitDefaults(e);
    const uint32_t wakes = 1800;
    for (uint32_t w = 0; w < wakes; ++w) {
        accumulateCoulomb(e, 1.0f, 0.0f);
    }
    TEST_ASSERT_EQUAL_UINT32(1800, e.chargedMilliAmpHours);
    TEST_ASSERT_EQUAL_UINT32(0, e.dischargedMilliAmpHours);
    TEST_ASSERT_EQUAL_UINT32(1, milliAhToAh(e.chargedMilliAmpHours));

    MockHal mock;
    mock.voltage = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    mock.current = 1.0f;
    mock.loadCurrent = 0.0f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::BULK);
    ChargerController c(mock.hal());
    c.begin();
    settle(c, mock, 8);
    runCycles(c, mock, wakes);
    TEST_ASSERT_EQUAL_UINT32(1, c.getChargedAh());
    TEST_ASSERT_EQUAL_UINT32(0, c.getDischargedAh());
}

void test_coulomb_negative_increments_discharged_only(void)
{
    EepromData e{};
    eepromInitDefaults(e);
    const uint32_t wakes = 1800;
    for (uint32_t w = 0; w < wakes; ++w) {
        accumulateCoulomb(e, 0.0f, 1.0f);
    }
    TEST_ASSERT_EQUAL_UINT32(0, e.chargedMilliAmpHours);
    TEST_ASSERT_EQUAL_UINT32(1800, e.dischargedMilliAmpHours);
    TEST_ASSERT_EQUAL_UINT32(1, milliAhToAh(e.dischargedMilliAmpHours));

    MockHal mock;
    mock.voltage = RECHARGE_VOLTAGE + 0.3f;
    mock.current = 0.0f;
    mock.loadCurrent = 1.0f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    settle(c, mock, 8);
    runCycles(c, mock, wakes);
    TEST_ASSERT_EQUAL_UINT32(0, c.getChargedAh());
    TEST_ASSERT_EQUAL_UINT32(1, c.getDischargedAh());
}

void test_coulomb_independent_counters(void)
{
    EepromData e{};
    eepromInitDefaults(e);
    accumulateCoulomb(e, 1.0f, 0.0f);
    uint32_t charged = e.chargedMilliAmpHours;
    accumulateCoulomb(e, 0.0f, 1.0f);
    TEST_ASSERT_EQUAL_UINT32(charged, e.chargedMilliAmpHours);
    TEST_ASSERT_TRUE(e.dischargedMilliAmpHours > 0);
}

void test_coulomb_uint32_rounding(void)
{
    EepromData e{};
    eepromInitDefaults(e);
    const float net = 2.88f;
    accumulateCoulomb(e, net, 0.0f);
    const float mAh = net * (SLEEP_INTERVAL_MS / 3600000.0f);
    const uint32_t expected = (uint32_t)(mAh * 1000.0f + 0.5f);
    TEST_ASSERT_EQUAL_UINT32(expected, e.chargedMilliAmpHours);
    TEST_ASSERT_TRUE(expected >= 1);
}

void test_coulomb_safety_early_return_does_not_count(void)
{
    MockHal mock;
    mock.voltage = MAX_CHARGE_VOLTAGE + 1.0f;
    mock.current = 2.0f;
    mock.loadCurrent = 0.0f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    runCycles(c, mock, FAULT_DEBOUNCE_CYCLES + 2);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ChargeState::FAULT), static_cast<int>(c.getState()));
    TEST_ASSERT_EQUAL_UINT32(0, c.getEeprom().chargedMilliAmpHours);
}

void run_coulomb_tests(void)
{
    RUN_TEST(test_coulomb_deadband_no_accumulate);
    RUN_TEST(test_coulomb_one_amp_1800_wakes);
    RUN_TEST(test_coulomb_negative_increments_discharged_only);
    RUN_TEST(test_coulomb_independent_counters);
    RUN_TEST(test_coulomb_uint32_rounding);
    RUN_TEST(test_coulomb_safety_early_return_does_not_count);
}
