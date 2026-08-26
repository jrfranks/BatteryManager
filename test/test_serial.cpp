#include "mock_hal.h"
#include "serial_parse.h"
#include "unity.h"
#include <string.h>

static int asCmd(Command c) { return static_cast<int>(c); }
static int asInt(ChargeState s) { return static_cast<int>(s); }

void test_serial_simple_commands(void)
{
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Help), asCmd(parseCommandLine("help").cmd));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Help), asCmd(parseCommandLine("?").cmd));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Status), asCmd(parseCommandLine("status").cmd));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Start), asCmd(parseCommandLine("start").cmd));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Stop), asCmd(parseCommandLine("stop").cmd));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Reset), asCmd(parseCommandLine("reset").cmd));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Dump), asCmd(parseCommandLine("dump").cmd));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Json), asCmd(parseCommandLine("json").cmd));
}

void test_serial_cal_commands(void)
{
    ParsedCommand v = parseCommandLine("cal v=+0.12");
    TEST_ASSERT_EQUAL_INT(asCmd(Command::CalV), asCmd(v.cmd));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.12f, v.value);

    ParsedCommand i = parseCommandLine("cal i=-0.05");
    TEST_ASSERT_EQUAL_INT(asCmd(Command::CalI), asCmd(i.cmd));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.05f, i.value);

    TEST_ASSERT_EQUAL_INT(asCmd(Command::Unknown), asCmd(parseCommandLine("cal v=").cmd));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Unknown), asCmd(parseCommandLine("cal v=abc").cmd));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Unknown), asCmd(parseCommandLine("cal v=1.0x").cmd));
}

void test_serial_execute_cal(void)
{
    MockHal mock;
    mock.voltage = RECHARGE_VOLTAGE + 0.3f;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    executeCommand(parseCommandLine("cal v=+0.12"), c);
    TEST_ASSERT_EQUAL_INT16(12, c.getVoltageCal());
    executeCommand(parseCommandLine("cal i=-0.05"), c);
    TEST_ASSERT_EQUAL_INT16(-5, c.getCurrentCal());
}

void test_serial_cal_clamp_and_nonfinite(void)
{
    const int16_t maxV = roundToCalTicks(CAL_OFFSET_MAX_V);
    const int16_t maxI = roundToCalTicks(CAL_OFFSET_MAX_A);
    TEST_ASSERT_EQUAL_INT16(50, maxV);
    TEST_ASSERT_EQUAL_INT16(50, maxI);

    {
        MockHal mock;
        mock.voltage = RECHARGE_VOLTAGE + 0.3f;
        plantValidEeprom(mock, ChargeState::IDLE);
        ChargerController c(mock.hal());
        c.begin();
        executeCommand(parseCommandLine("cal v=+5.0"), c);
        TEST_ASSERT_EQUAL_INT16(maxV, c.getVoltageCal());
        executeCommand(parseCommandLine("cal i=-5.0"), c);
        TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(-maxI), c.getCurrentCal());

        const int16_t vBefore = c.getVoltageCal();
        const int16_t iBefore = c.getCurrentCal();
        ParsedCommand nanCmd = parseCommandLine("cal v=nan");
        TEST_ASSERT_EQUAL_INT(asCmd(Command::Unknown), asCmd(nanCmd.cmd));
        executeCommand(nanCmd, c);
        TEST_ASSERT_EQUAL_INT16(vBefore, c.getVoltageCal());
        TEST_ASSERT_EQUAL_INT16(iBefore, c.getCurrentCal());

        ParsedCommand infCmd = parseCommandLine("cal v=inf");
        TEST_ASSERT_EQUAL_INT(asCmd(Command::Unknown), asCmd(infCmd.cmd));
        executeCommand(infCmd, c);
        TEST_ASSERT_EQUAL_INT16(vBefore, c.getVoltageCal());

        ParsedCommand ninfCmd = parseCommandLine("cal i=-inf");
        TEST_ASSERT_EQUAL_INT(asCmd(Command::Unknown), asCmd(ninfCmd.cmd));
        executeCommand(ninfCmd, c);
        TEST_ASSERT_EQUAL_INT16(iBefore, c.getCurrentCal());
    }

    {
        MockHal mock;
        mock.voltage = RECHARGE_VOLTAGE + 0.3f;
        plantValidEeprom(mock, ChargeState::IDLE);
        ChargerController c(mock.hal());
        c.begin();
        executeCommand(parseCommandLine("cal v=+0.40"), c);
        TEST_ASSERT_EQUAL_INT16(40, c.getVoltageCal());
        executeCommand(parseCommandLine("cal v=+0.40"), c);
        TEST_ASSERT_EQUAL_INT16(maxV, c.getVoltageCal());
        executeCommand(parseCommandLine("cal i=-0.40"), c);
        TEST_ASSERT_EQUAL_INT16(-40, c.getCurrentCal());
        executeCommand(parseCommandLine("cal i=-0.40"), c);
        TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(-maxI), c.getCurrentCal());
    }
}

void test_serial_unknown_and_empty(void)
{
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Unknown), asCmd(parseCommandLine("nope").cmd));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::None), asCmd(parseCommandLine("").cmd));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::None), asCmd(parseCommandLine(nullptr).cmd));
}

void test_serial_truncation_31_chars(void)
{
    SerialCmdBuffer line;
    ParsedCommand parsed{};
    for (int n = 0; n < 40; ++n) {
        TEST_ASSERT_EQUAL_INT(
          static_cast<int>(SerialCmdBuffer::FeedResult::Pending), static_cast<int>(line.feed('A', parsed)));
    }
    TEST_ASSERT_EQUAL_INT(
      static_cast<int>(SerialCmdBuffer::FeedResult::Line), static_cast<int>(line.feed('\n', parsed)));
    TEST_ASSERT_EQUAL_UINT32(31, (uint32_t)strlen(line.buf));
    TEST_ASSERT_EQUAL_INT(asCmd(Command::Unknown), asCmd(parsed.cmd));
}

void test_serial_start_stop_master_enable(void)
{
    MockHal mock;
    mock.voltage = NOMINAL_VOLTAGE;
    mock.masterEnable = false;
    plantValidEeprom(mock, ChargeState::IDLE);
    ChargerController c(mock.hal());
    c.begin();
    executeCommand(parseCommandLine("start"), c);
    TEST_ASSERT_TRUE(mock.masterEnable);
    executeCommand(parseCommandLine("stop"), c);
    TEST_ASSERT_FALSE(mock.masterEnable);
}

void test_serial_reset_only_leaves_fault(void)
{
    {
        MockHal mock;
        mock.voltage = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
        mock.tempC = 25.0f;
        plantValidEeprom(mock, ChargeState::BULK);
        ChargerController c(mock.hal());
        c.begin();
        settle(c, mock, 4);
        TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
        executeCommand(parseCommandLine("reset"), c);
        TEST_ASSERT_EQUAL_INT(asInt(ChargeState::BULK), asInt(c.getState()));
    }
    {
        MockHal mockF;
        mockF.voltage = 2.0f;
        plantValidEeprom(mockF, ChargeState::INIT);
        ChargerController cf(mockF.hal());
        cf.begin();
        runCycles(cf, mockF, 1);
        TEST_ASSERT_EQUAL_INT(asInt(ChargeState::FAULT), asInt(cf.getState()));
        executeCommand(parseCommandLine("reset"), cf);
        TEST_ASSERT_EQUAL_INT(asInt(ChargeState::RECOVERY), asInt(cf.getState()));
    }
}

void run_serial_tests(void)
{
    RUN_TEST(test_serial_simple_commands);
    RUN_TEST(test_serial_cal_commands);
    RUN_TEST(test_serial_execute_cal);
    RUN_TEST(test_serial_cal_clamp_and_nonfinite);
    RUN_TEST(test_serial_unknown_and_empty);
    RUN_TEST(test_serial_truncation_31_chars);
    RUN_TEST(test_serial_start_stop_master_enable);
    RUN_TEST(test_serial_reset_only_leaves_fault);
}
