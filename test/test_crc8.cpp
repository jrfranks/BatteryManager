#include "crc8.h"
#include "persistence.h"
#include "unity.h"
#include <string.h>

void test_crc8_empty_buffer(void) { TEST_ASSERT_EQUAL_UINT8(0, crc8(nullptr, 0)); }

void test_crc8_known_vector(void)
{
    const uint8_t msg[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_UINT8(0xA1, crc8(msg, 9));
}

void test_crc8_detects_single_bit_flip(void)
{
    uint8_t buf[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t good = crc8(buf, 4);
    buf[1] ^= 0x04;
    TEST_ASSERT_NOT_EQUAL(good, crc8(buf, 4));
}

void test_crc8_eeprom_save_then_validate(void)
{
    EepromData e{};
    eepromInitDefaults(e);
    e.chargedMilliAmpHours = 1234;
    eepromFinalizeCrc(e);
    TEST_ASSERT_EQUAL_UINT8(eepromCrc(e), e.crc8);
    TEST_ASSERT_TRUE(eepromIsValid(e));
}

static void assert_mutating_field_fails(void (*mutate)(EepromData &))
{
    EepromData e{};
    eepromInitDefaults(e);
    eepromFinalizeCrc(e);
    TEST_ASSERT_TRUE(eepromIsValid(e));
    mutate(e);
    TEST_ASSERT_FALSE(eepromIsValid(e));
}

void test_crc8_mutating_any_field_fails_validation(void)
{
    assert_mutating_field_fails([](EepromData &e) { e.magic ^= 1; });
    assert_mutating_field_fails([](EepromData &e) { e.version ^= 1; });
    assert_mutating_field_fails([](EepromData &e) { e.lastState ^= 1; });
    assert_mutating_field_fails([](EepromData &e) { e.chargedMilliAmpHours ^= 1u; });
    assert_mutating_field_fails([](EepromData &e) { e.dischargedMilliAmpHours ^= 1u; });
    assert_mutating_field_fails([](EepromData &e) { e.vCalOffset = static_cast<int16_t>(e.vCalOffset ^ 1); });
    assert_mutating_field_fails([](EepromData &e) { e.iCalOffset = static_cast<int16_t>(e.iCalOffset ^ 1); });
}

void run_crc8_tests(void)
{
    RUN_TEST(test_crc8_empty_buffer);
    RUN_TEST(test_crc8_known_vector);
    RUN_TEST(test_crc8_detects_single_bit_flip);
    RUN_TEST(test_crc8_eeprom_save_then_validate);
    RUN_TEST(test_crc8_mutating_any_field_fails_validation);
}
