#include <unity.h>

void run_crc8_tests(void);
void run_sensor_math_tests(void);
void run_temp_comp_tests(void);
void run_control_law_tests(void);
void run_coulomb_tests(void);
void run_safety_tests(void);
void run_fsm_tests(void);
void run_eeprom_tests(void);
void run_led_tests(void);
void run_serial_tests(void);
void run_telemetry_tests(void);
void run_charger_cycle_tests(void);

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    run_crc8_tests();
    run_sensor_math_tests();
    run_temp_comp_tests();
    run_control_law_tests();
    run_coulomb_tests();
    run_safety_tests();
    run_fsm_tests();
    run_eeprom_tests();
    run_led_tests();
    run_serial_tests();
    run_telemetry_tests();
    run_charger_cycle_tests();
    return UNITY_END();
}
