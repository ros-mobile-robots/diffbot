// Unit tests of WheelControl (lib/wheel_control/wheel_control.h) on the PC, with a fake base
// controller and timeout log (test/fakes/fake_base_controller.h). Run them with:
//   pio test -e native
#include <unity.h>

#include <stdint.h>

#include "../fakes/fake_base_controller.h"
#include "wheel_command.h"
#include "wheel_control.h"

typedef diffbot::WheelControl<FakeBaseController, FakeTimeoutLog> Control;

// Fixed inputs of these tests, independent of the robot's settings. The times in the tests are
// written out for them: a command expires 1000 ms after it arrived, and a control step is due
// 50 ms after the last one
constexpr diffbot::WheelControlConfig TEST_CONFIG = {
    1000, // command_timeout_ms
    50,   // control_period_ms
};
constexpr diffbot::WheelCommand TEST_COMMAND = {1.25f, -2.5f};

void setUp() {}
void tearDown() {}

void test_wheel_control_reads_without_driving_before_first_command()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});

    control.step(50);
    control.step(100);

    TEST_ASSERT_EQUAL_STRING("read read", controller.calls.c_str());
}

void test_wheel_control_stores_targets_without_driving_on_command()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});

    control.receiveCommand(TEST_COMMAND, 100);

    TEST_ASSERT_EQUAL_STRING("setWheelTargets", controller.calls.c_str());
    TEST_ASSERT_EQUAL_FLOAT(TEST_COMMAND.left_rad_s, controller.targets.left_rad_s);
    TEST_ASSERT_EQUAL_FLOAT(TEST_COMMAND.right_rad_s, controller.targets.right_rad_s);
}

void test_wheel_control_reads_before_writing_when_command_fresh()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 100);
    controller.clearCalls();

    control.step(150);

    TEST_ASSERT_EQUAL_STRING("read write", controller.calls.c_str());
}

void test_wheel_control_stops_before_logging_when_command_expires()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 100);
    control.step(150);
    controller.clearCalls();

    control.step(1100);

    TEST_ASSERT_EQUAL_STRING("eStop log read", controller.calls.c_str());
    TEST_ASSERT_EQUAL_UINT(1, controller.motor_outputs_at_log.size());
    TEST_ASSERT_EQUAL_INT(0, controller.motor_outputs_at_log[0]);
}

void test_wheel_control_stops_and_logs_when_first_command_already_stale()
{
    // The first command arrives during setup() and is too old at the first loop()
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 100);
    controller.clearCalls();

    control.step(1100);

    TEST_ASSERT_EQUAL_STRING("eStop log read", controller.calls.c_str());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, controller.targets.left_rad_s);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, controller.targets.right_rad_s);
}

void test_wheel_control_stops_and_logs_when_new_command_stale_after_timeout()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 100);
    control.step(1100);
    control.receiveCommand(TEST_COMMAND, 1200);
    controller.clearCalls();

    control.step(2200);

    TEST_ASSERT_EQUAL_STRING("eStop log read", controller.calls.c_str());
}

void test_wheel_control_resumes_when_new_command_has_same_timestamp()
{
    // After a whole cycle of millis(), about 49.7 days, a new command arrives at the same time
    // as the old one
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 100);
    control.step(1100);
    control.receiveCommand(TEST_COMMAND, 100);
    controller.clearCalls();

    control.step(100);

    TEST_ASSERT_EQUAL_STRING("read write", controller.calls.c_str());
}

void test_wheel_control_remains_inhibited_after_complete_clock_cycle()
{
    // A whole cycle of millis() later, without a new command: the old command's time looks new
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 100);
    control.step(1100);
    controller.clearCalls();

    control.step(100);

    TEST_ASSERT_EQUAL_STRING("read", controller.calls.c_str());
}

void test_wheel_control_stops_when_expiry_precedes_next_control_step()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 100);
    control.step(1090);
    controller.clearCalls();

    control.step(1100);

    TEST_ASSERT_EQUAL_STRING("eStop log", controller.calls.c_str());
    TEST_ASSERT_EQUAL_UINT(1, controller.motor_outputs_at_log.size());
    TEST_ASSERT_EQUAL_INT(0, controller.motor_outputs_at_log[0]);
}

void test_wheel_control_reads_without_writing_while_timed_out()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 100);
    control.step(1100);
    controller.clearCalls();

    control.step(1150);
    control.step(1200);

    TEST_ASSERT_EQUAL_STRING("read read", controller.calls.c_str());
}

void test_wheel_control_does_not_repeat_stop_or_log_without_new_command()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 100);
    control.step(1100);
    controller.clearCalls();

    control.step(1101);
    control.step(1102);

    TEST_ASSERT_EQUAL_STRING("", controller.calls.c_str());
}

void test_wheel_control_resumes_on_schedule_after_fresh_command()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 100);
    control.step(1100);
    control.receiveCommand(TEST_COMMAND, 1120);
    controller.clearCalls();

    control.step(1150);

    TEST_ASSERT_EQUAL_STRING("read write", controller.calls.c_str());
}

void test_wheel_control_waits_until_control_period_has_elapsed()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 0);
    control.step(50);
    controller.clearCalls();

    control.step(99);

    TEST_ASSERT_EQUAL_STRING("", controller.calls.c_str());
}

void test_wheel_control_runs_one_step_after_delayed_loop()
{
    // loop() was held up for many control periods: the steps it missed don't run afterwards
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 0);
    control.step(50);
    controller.clearCalls();

    control.step(999);
    control.step(999);

    TEST_ASSERT_EQUAL_STRING("read write", controller.calls.c_str());
}

void test_wheel_control_preserves_schedule_when_commands_arrive()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, 0);
    control.step(50);
    control.receiveCommand(TEST_COMMAND, 99);
    controller.clearCalls();

    control.step(100);

    TEST_ASSERT_EQUAL_STRING("read write", controller.calls.c_str());
}

void test_wheel_control_runs_on_schedule_across_clock_wrap()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, UINT32_MAX - 20);
    control.step(UINT32_MAX - 20);
    controller.clearCalls();

    control.step(28);
    control.step(29);

    TEST_ASSERT_EQUAL_STRING("read write", controller.calls.c_str());
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_wheel_control_reads_without_driving_before_first_command);
    RUN_TEST(test_wheel_control_stores_targets_without_driving_on_command);
    RUN_TEST(test_wheel_control_reads_before_writing_when_command_fresh);
    RUN_TEST(test_wheel_control_stops_before_logging_when_command_expires);
    RUN_TEST(test_wheel_control_stops_and_logs_when_first_command_already_stale);
    RUN_TEST(test_wheel_control_stops_and_logs_when_new_command_stale_after_timeout);
    RUN_TEST(test_wheel_control_resumes_when_new_command_has_same_timestamp);
    RUN_TEST(test_wheel_control_remains_inhibited_after_complete_clock_cycle);
    RUN_TEST(test_wheel_control_stops_when_expiry_precedes_next_control_step);
    RUN_TEST(test_wheel_control_reads_without_writing_while_timed_out);
    RUN_TEST(test_wheel_control_does_not_repeat_stop_or_log_without_new_command);
    RUN_TEST(test_wheel_control_resumes_on_schedule_after_fresh_command);
    RUN_TEST(test_wheel_control_waits_until_control_period_has_elapsed);
    RUN_TEST(test_wheel_control_runs_one_step_after_delayed_loop);
    RUN_TEST(test_wheel_control_preserves_schedule_when_commands_arrive);
    RUN_TEST(test_wheel_control_runs_on_schedule_across_clock_wrap);
    return UNITY_END();
}
