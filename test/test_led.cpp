#include "Config.h"
#include "led_pattern.h"
#include "unity.h"

void test_led_idle(void)
{
    const uint8_t period[] = {1, 0, 0, 0, 0, 0, 0, 0};
    for (uint32_t w = 0; w < 16; ++w) {
        TEST_ASSERT_EQUAL_INT(period[w % 8u], ledOnForState(ChargeState::IDLE, w) ? 1 : 0);
    }
}

void test_led_bulk(void)
{
    const uint8_t period[] = {1, 1, 0};
    for (uint32_t w = 0; w < 12; ++w) {
        TEST_ASSERT_EQUAL_INT(period[w % 3u], ledOnForState(ChargeState::BULK, w) ? 1 : 0);
    }
}

void test_led_absorption(void)
{
    const uint8_t period[] = {1, 1, 0, 0};
    for (uint32_t w = 0; w < 12; ++w) {
        TEST_ASSERT_EQUAL_INT(period[w % 4u], ledOnForState(ChargeState::ABSORPTION, w) ? 1 : 0);
    }
}

void test_led_float(void)
{
    const uint8_t period[] = {1, 0, 0, 0, 0, 0};
    for (uint32_t w = 0; w < 12; ++w) {
        TEST_ASSERT_EQUAL_INT(period[w % 6u], ledOnForState(ChargeState::FLOAT, w) ? 1 : 0);
    }
}

void test_led_fault(void)
{
    const uint8_t period[] = {0, 1};
    for (uint32_t w = 0; w < 8; ++w) {
        TEST_ASSERT_EQUAL_INT(period[w % 2u], ledOnForState(ChargeState::FAULT, w) ? 1 : 0);
    }
}

void test_led_precharge(void)
{
    const uint8_t period[] = {1, 0, 0, 0, 0};
    for (uint32_t w = 0; w < 10; ++w) {
        TEST_ASSERT_EQUAL_INT(period[w % 5u], ledOnForState(ChargeState::PRECHARGE, w) ? 1 : 0);
    }
}

void test_led_init_recovery_default_off(void)
{
    for (uint32_t w = 0; w < 16; ++w) {
        TEST_ASSERT_FALSE(ledOnForState(ChargeState::INIT, w));
        TEST_ASSERT_FALSE(ledOnForState(ChargeState::RECOVERY, w));
    }
}

void run_led_tests(void)
{
    RUN_TEST(test_led_idle);
    RUN_TEST(test_led_bulk);
    RUN_TEST(test_led_absorption);
    RUN_TEST(test_led_float);
    RUN_TEST(test_led_fault);
    RUN_TEST(test_led_precharge);
    RUN_TEST(test_led_init_recovery_default_off);
}
