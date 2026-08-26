#include "Config.h"
#include "charge_fsm.h"
#include "mock_hal.h"
#include "persistence.h"
#include "unity.h"
#include <string.h>

static int asInt(ChargeState s) { return static_cast<int>(s); }

void test_eeprom_virgin_loads_defaults_and_saves(void)
{
    MockHal mock;
    memset(mock.eepromMem, 0xFF, sizeof(mock.eepromMem));
    mock.voltage = NOMINAL_VOLTAGE;
    mock.tempC = 25.0f;
    ChargerController c(mock.hal());
    c.begin();
    TEST_ASSERT_TRUE(mock.saveCount >= 1);
    TEST_ASSERT_EQUAL_UINT16(EEPROM_MAGIC, c.getEeprom().magic);
    TEST_ASSERT_EQUAL_UINT8(EEPROM_VERSION, c.getEeprom().version);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ChargeState::IDLE), c.getEeprom().lastState);
    TEST_ASSERT_EQUAL_UINT32(0, c.getEeprom().chargedMilliAmpHours);
    TEST_ASSERT_EQUAL_INT16(0, c.getEeprom().vCalOffset);
    TEST_ASSERT_TRUE(eepromIsValid(c.getEeprom()));
}

void test_eeprom_bad_magic_version_crc(void)
{
    {
        MockHal mock;
        EepromData bad{};
        eepromInitDefaults(bad);
        eepromFinalizeCrc(bad);
        bad.magic = 0x1111;
        mock.plantEeprom(bad);
        mock.voltage = NOMINAL_VOLTAGE;
        ChargerController c(mock.hal());
        c.begin();
        TEST_ASSERT_EQUAL_UINT16(EEPROM_MAGIC, c.getEeprom().magic);
    }
    {
        EepromData bad{};
        eepromInitDefaults(bad);
        bad.version = static_cast<uint8_t>(EEPROM_VERSION + 1);
        eepromFinalizeCrc(bad);
        MockHal mock2;
        mock2.plantEeprom(bad);
        mock2.voltage = NOMINAL_VOLTAGE;
        ChargerController c2(mock2.hal());
        c2.begin();
        TEST_ASSERT_EQUAL_UINT8(EEPROM_VERSION, c2.getEeprom().version);
    }
    {
        EepromData bad{};
        eepromInitDefaults(bad);
        eepromFinalizeCrc(bad);
        bad.crc8 ^= 0xFF;
        MockHal mock3;
        mock3.plantEeprom(bad);
        mock3.voltage = NOMINAL_VOLTAGE;
        ChargerController c3(mock3.hal());
        c3.begin();
        TEST_ASSERT_TRUE(eepromIsValid(c3.getEeprom()));
    }
}

void test_eeprom_round_trip(void)
{
    MockHal mock;
    plantValidEeprom(mock, ChargeState::FLOAT, 5000, 1200, 3, -2);
    mock.voltage = FLOAT_VOLTAGE;
    mock.tempC = 25.0f;
    ChargerController c(mock.hal());
    c.begin();
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FLOAT), asInt(c.getState()));
    TEST_ASSERT_EQUAL_UINT32(5000, c.getEeprom().chargedMilliAmpHours);
    TEST_ASSERT_EQUAL_UINT32(1200, c.getEeprom().dischargedMilliAmpHours);
    TEST_ASSERT_EQUAL_INT16(3, c.getVoltageCal());
    TEST_ASSERT_EQUAL_INT16(-2, c.getCurrentCal());
}

void test_eeprom_throttle_non_critical(void)
{
    MockHal mock;
    mock.voltage = PRECHARGE_VOLTAGE - 0.2f;
    if (mock.voltage < MIN_OPERATING_VOLTAGE) {
        mock.voltage = (MIN_OPERATING_VOLTAGE + PRECHARGE_VOLTAGE) * 0.5f;
    }
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    unsigned saves = mock.saveCount;

    runCycles(c, mock, 2);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::PRECHARGE), asInt(c.getState()));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ChargeState::PRECHARGE), c.getEeprom().lastState);
    TEST_ASSERT_EQUAL_UINT32(saves, mock.saveCount);

    EepromData disk = mock.peekEeprom();
    TEST_ASSERT_NOT_EQUAL(static_cast<uint8_t>(ChargeState::PRECHARGE), disk.lastState);
}

void test_eeprom_immediate_on_critical(void)
{
    MockHal mock;
    mock.voltage = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    unsigned saves = mock.saveCount;
    settle(c, mock, 16);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
    TEST_ASSERT_TRUE(mock.saveCount > saves);

    saves = mock.saveCount;
    mock.voltage = ABSORPTION_VOLTAGE;
    mock.current = MAX_CHARGE_CURRENT * 0.4f;
    settle(c, mock, 30);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::ABSORPTION), asInt(c.getState()));
    TEST_ASSERT_TRUE(mock.saveCount > saves);
}

void test_eeprom_immediate_on_fault(void)
{
    MockHal mockF;
    mockF.voltage = MAX_CHARGE_VOLTAGE + 2.0f;
    mockF.tempC = 25.0f;
    plantValidEeprom(mockF, ChargeState::IDLE);
    ChargerController cf(mockF.hal());
    cf.begin();
    unsigned saves = mockF.saveCount;
    runCycles(cf, mockF, FAULT_DEBOUNCE_CYCLES);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(cf.getState()));
    TEST_ASSERT_TRUE(mockF.saveCount > saves);
}

void test_eeprom_bulk_stop_stays_bulk(void)
{
    MockHal mock;
    mock.voltage = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::BULK);
    ChargerController c(mock.hal());
    c.begin();
    runCycles(c, mock, 4);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
    c.setMasterEnable(false);
    c.forceOutputsOff();
    runCycles(c, mock, 20);
    TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
}

void test_eeprom_critical_transition_helper(void)
{
    TEST_ASSERT_TRUE(isCriticalTransition(ChargeState::IDLE, ChargeState::FAULT));
    TEST_ASSERT_TRUE(isCriticalTransition(ChargeState::IDLE, ChargeState::BULK));
    TEST_ASSERT_TRUE(isCriticalTransition(ChargeState::BULK, ChargeState::ABSORPTION));
    TEST_ASSERT_TRUE(isCriticalTransition(ChargeState::BULK, ChargeState::IDLE));
    TEST_ASSERT_FALSE(isCriticalTransition(ChargeState::IDLE, ChargeState::PRECHARGE));
    TEST_ASSERT_FALSE(isCriticalTransition(ChargeState::ABSORPTION, ChargeState::FLOAT));
    // BULK→IDLE is not produced by the live FSM (stop leaves BULK); the helper still pins the save rule.
}

void test_eeprom_throttle_interval_wake_save(void)
{
    MockHal mock;
    mock.voltage = RECHARGE_VOLTAGE + 0.4f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    unsigned saves = mock.saveCount;
    runCycles(c, mock, EEPROM_SAVE_INTERVAL + 2);
    TEST_ASSERT_TRUE(mock.saveCount > saves);
}

void run_eeprom_tests(void)
{
    RUN_TEST(test_eeprom_virgin_loads_defaults_and_saves);
    RUN_TEST(test_eeprom_bad_magic_version_crc);
    RUN_TEST(test_eeprom_round_trip);
    RUN_TEST(test_eeprom_throttle_non_critical);
    RUN_TEST(test_eeprom_immediate_on_critical);
    RUN_TEST(test_eeprom_immediate_on_fault);
    RUN_TEST(test_eeprom_bulk_stop_stays_bulk);
    RUN_TEST(test_eeprom_critical_transition_helper);
    RUN_TEST(test_eeprom_throttle_interval_wake_save);
}
