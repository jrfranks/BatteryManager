#include "Config.h"
#include "control_law.h"
#include "mock_hal.h"
#include "unity.h"

static void assert_zero_output(const ControlLawOutput &o)
{
    TEST_ASSERT_EQUAL_UINT8(0, o.duty);
    TEST_ASSERT_FALSE(o.enable);
}

void test_control_idle_fault_master_off_zero_duty(void)
{
    assert_zero_output(applyControlLaw(ChargeState::IDLE, true, 13.0f, 0.0f, 13.5f, 0.0f, 100, 5.0f));
    assert_zero_output(applyControlLaw(ChargeState::FAULT, true, 13.0f, 1.0f, 14.4f, 5.0f, 100, 5.0f));
    assert_zero_output(applyControlLaw(ChargeState::BULK, false, 12.0f, 0.0f, 14.4f, 5.0f, 100, 5.0f));
}

void test_control_voltage_mode_ki_zero(void)
{
    ControlLawOutput a =
      applyControlLaw(ChargeState::ABSORPTION, true, ABSORPTION_VOLTAGE, 1.0f, ABSORPTION_VOLTAGE, 5.0f, 40, 12.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 12.0f, a.pwmIntegral);

    ControlLawOutput f = applyControlLaw(ChargeState::FLOAT, true, FLOAT_VOLTAGE, 0.2f, FLOAT_VOLTAGE, 1.0f, 20, -8.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -8.0f, f.pwmIntegral);
}

void test_control_current_mode_integral_clamped(void)
{
    float integ = 0.0f;
    uint8_t duty = 0;
    for (int n = 0; n < 200; ++n) {
        ControlLawOutput o =
          applyControlLaw(ChargeState::BULK, true, 12.0f, 0.0f, ABSORPTION_VOLTAGE, MAX_CHARGE_CURRENT, duty, integ);
        integ = o.pwmIntegral;
        duty = o.duty;
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 30.0f, integ);

    integ = 0.0f;
    duty = 40;
    for (int n = 0; n < 400; ++n) {
        ControlLawOutput o = applyControlLaw(ChargeState::PRECHARGE, true, 11.0f, MAX_CHARGE_CURRENT * 4.0f,
          ABSORPTION_VOLTAGE, PRECHARGE_CURRENT, duty, integ);
        integ = o.pwmIntegral;
        duty = o.duty;
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -30.0f, integ);
}

void test_control_slew_limit(void)
{
    ControlLawOutput up =
      applyControlLaw(ChargeState::ABSORPTION, true, 0.0f, 0.0f, ABSORPTION_VOLTAGE, MAX_CHARGE_CURRENT, 40, 0.0f);
    TEST_ASSERT_EQUAL_INT16((int16_t)PWM_SLEW_LIMIT, (int16_t)up.duty - 40);

    ControlLawOutput down =
      applyControlLaw(ChargeState::ABSORPTION, true, 20.0f, 0.0f, ABSORPTION_VOLTAGE, 0.0f, 200, 0.0f);
    TEST_ASSERT_EQUAL_INT16((int16_t)PWM_SLEW_LIMIT, 200 - (int16_t)down.duty);
}

void test_control_duty_clamped(void)
{
    uint8_t duty = 250;
    float integ = 30.0f;
    for (int n = 0; n < 8; ++n) {
        ControlLawOutput o =
          applyControlLaw(ChargeState::ABSORPTION, true, 0.0f, 0.0f, ABSORPTION_VOLTAGE, 0.0f, duty, integ);
        duty = o.duty;
        integ = o.pwmIntegral;
    }
    TEST_ASSERT_EQUAL_UINT8((uint8_t)PWM_MAX_DUTY, duty);

    duty = 2;
    integ = -30.0f;
    for (int n = 0; n < 8; ++n) {
        ControlLawOutput o =
          applyControlLaw(ChargeState::ABSORPTION, true, 40.0f, 0.0f, ABSORPTION_VOLTAGE, 0.0f, duty, integ);
        duty = o.duty;
        integ = o.pwmIntegral;
    }
    TEST_ASSERT_EQUAL_UINT8(0, duty);
}

void test_control_enable_duty_or_precharge(void)
{
    ControlLawOutput pre = applyControlLaw(
      ChargeState::PRECHARGE, true, 11.0f, PRECHARGE_CURRENT, ABSORPTION_VOLTAGE, PRECHARGE_CURRENT, 0, 0.0f);
    TEST_ASSERT_TRUE(pre.enable);

    ControlLawOutput bulkLow = applyControlLaw(
      ChargeState::BULK, true, 12.0f, MAX_CHARGE_CURRENT, ABSORPTION_VOLTAGE, MAX_CHARGE_CURRENT, 3, 0.0f);
    TEST_ASSERT_FALSE(bulkLow.enable);
    TEST_ASSERT_TRUE(bulkLow.duty <= 5);

    uint8_t duty = 0;
    float integ = 0.0f;
    bool high = false;
    for (int n = 0; n < 16; ++n) {
        ControlLawOutput o =
          applyControlLaw(ChargeState::BULK, true, 12.0f, 0.0f, ABSORPTION_VOLTAGE, MAX_CHARGE_CURRENT, duty, integ);
        duty = o.duty;
        integ = o.pwmIntegral;
        if (duty > 5) {
            TEST_ASSERT_TRUE(o.enable);
            high = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(high);
}

void test_control_integrator_reset_on_transition(void)
{
    MockHal mock;
    mock.voltage = (PRECHARGE_VOLTAGE + RECHARGE_VOLTAGE) * 0.5f;
    mock.current = 0.0f;
    mock.tempC = 25.0f;
    plantValidEeprom(mock, ChargeState::BULK);
    ChargerController c(mock.hal());
    c.begin();
    runCycles(c, mock, 8);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ChargeState::BULK), static_cast<int>(c.getState()));
    TEST_ASSERT_TRUE(c.getPwmIntegral() != 0.0f || c.getPWM() > 0);

    mock.voltage = ABSORPTION_VOLTAGE;
    mock.current = MAX_CHARGE_CURRENT * 0.5f;
    settle(c, mock, 30);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ChargeState::ABSORPTION), static_cast<int>(c.getState()));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, c.getPwmIntegral());
}

void run_control_law_tests(void)
{
    RUN_TEST(test_control_idle_fault_master_off_zero_duty);
    RUN_TEST(test_control_voltage_mode_ki_zero);
    RUN_TEST(test_control_current_mode_integral_clamped);
    RUN_TEST(test_control_slew_limit);
    RUN_TEST(test_control_duty_clamped);
    RUN_TEST(test_control_enable_duty_or_precharge);
    RUN_TEST(test_control_integrator_reset_on_transition);
}
