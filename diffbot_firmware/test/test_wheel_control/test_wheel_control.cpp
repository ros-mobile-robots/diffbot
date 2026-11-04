// Unit tests of WheelControl (lib/wheel_control/wheel_control.h) on the PC, with a fake base
// controller and timeout log (test/fakes/fake_base_controller.h). Run them with:
//   pio test -e native
#include <unity.h>

#include <stdint.h>

#include "../fakes/fake_base_controller.h"
#include "wheel_command.h"
#include "wheel_control.h"

typedef diffbot::WheelControl<FakeBaseController, FakeTimeoutLog> Control;

// The timing of these tests. Fixed values, independent of the robot's settings in
// diffbot_base_config.h, so that a change of the settings doesn't change what the tests check
constexpr uint32_t TEST_TIMEOUT_MS = 1000;
constexpr uint32_t TEST_CONTROL_PERIOD_MS = 50;
constexpr diffbot::WheelControlConfig TEST_CONFIG = {TEST_TIMEOUT_MS, TEST_CONTROL_PERIOD_MS};
constexpr diffbot::WheelCommand TEST_COMMAND = {1.25f, -2.5f};

// When a test's first command arrives, and the first time at which that command is too old
constexpr uint32_t ARRIVAL_MS = 100;
constexpr uint32_t DEADLINE_MS = ARRIVAL_MS + TEST_TIMEOUT_MS;

void setUp() {}
void tearDown() {}

void test_wheel_control_reads_without_driving_before_first_command()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});

    control.step(TEST_CONTROL_PERIOD_MS);
    control.step(2 * TEST_CONTROL_PERIOD_MS);

    TEST_ASSERT_EQUAL_STRING("read read", controller.calls.c_str());
}

void test_wheel_control_stores_targets_without_driving_on_command()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});

    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);

    TEST_ASSERT_EQUAL_STRING("setWheelTargets", controller.calls.c_str());
    TEST_ASSERT_EQUAL_FLOAT(TEST_COMMAND.left_rad_s, controller.targets.left_rad_s);
    TEST_ASSERT_EQUAL_FLOAT(TEST_COMMAND.right_rad_s, controller.targets.right_rad_s);
}

void test_wheel_control_reads_before_writing_when_command_fresh()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    controller.clearCalls();

    control.step(ARRIVAL_MS + TEST_CONTROL_PERIOD_MS);

    TEST_ASSERT_EQUAL_STRING("read write", controller.calls.c_str());
}

void test_wheel_control_stops_before_logging_when_command_expires()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    control.step(ARRIVAL_MS + TEST_CONTROL_PERIOD_MS);
    controller.clearCalls();

    control.step(DEADLINE_MS);

    TEST_ASSERT_EQUAL_STRING("eStop log read", controller.calls.c_str());
    TEST_ASSERT_EQUAL_UINT(1, controller.motor_outputs_at_log.size());
    TEST_ASSERT_EQUAL_INT(0, controller.motor_outputs_at_log[0]);
}

void test_wheel_control_stops_and_logs_when_first_command_already_stale()
{
    // The first command arrives during setup() and is too old at the first loop()
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    controller.clearCalls();

    control.step(DEADLINE_MS);

    TEST_ASSERT_EQUAL_STRING("eStop log read", controller.calls.c_str());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, controller.targets.left_rad_s);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, controller.targets.right_rad_s);
}

void test_wheel_control_stops_and_logs_when_new_command_stale_after_timeout()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    control.step(DEADLINE_MS);
    const uint32_t second_arrival_ms = DEADLINE_MS + 100;
    control.receiveCommand(TEST_COMMAND, second_arrival_ms);
    controller.clearCalls();

    control.step(second_arrival_ms + TEST_TIMEOUT_MS);

    TEST_ASSERT_EQUAL_STRING("eStop log read", controller.calls.c_str());
}

void test_wheel_control_resumes_when_new_command_has_same_timestamp()
{
    // After a whole cycle of millis(), about 49.7 days, a new command arrives at the same clock
    // value as the old one
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    control.step(DEADLINE_MS);
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    controller.clearCalls();

    control.step(ARRIVAL_MS);

    TEST_ASSERT_EQUAL_STRING("read write", controller.calls.c_str());
}

void test_wheel_control_stays_timed_out_after_complete_clock_cycle()
{
    // A whole cycle of millis() later, without a new command, the clock shows the old command's
    // arrival time again. That reading alone must not make the old command new
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    control.step(DEADLINE_MS);
    controller.clearCalls();

    control.step(ARRIVAL_MS);

    TEST_ASSERT_EQUAL_STRING("read", controller.calls.c_str());
}

void test_wheel_control_stops_when_expiry_precedes_next_control_step()
{
    // A control step 10 ms before the deadline: the next one is due only 40 ms after the deadline
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    const uint32_t last_step_ms = DEADLINE_MS - 10;
    control.step(last_step_ms);
    controller.clearCalls();

    control.step(DEADLINE_MS);

    TEST_ASSERT_EQUAL_STRING("eStop log", controller.calls.c_str());
    TEST_ASSERT_EQUAL_UINT(1, controller.motor_outputs_at_log.size());
    TEST_ASSERT_EQUAL_INT(0, controller.motor_outputs_at_log[0]);
}

void test_wheel_control_reads_without_writing_while_timed_out()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    control.step(DEADLINE_MS);
    controller.clearCalls();

    control.step(DEADLINE_MS + TEST_CONTROL_PERIOD_MS);
    control.step(DEADLINE_MS + 2 * TEST_CONTROL_PERIOD_MS);

    TEST_ASSERT_EQUAL_STRING("read read", controller.calls.c_str());
}

void test_wheel_control_does_not_repeat_stop_or_log_without_new_command()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    control.step(DEADLINE_MS);
    controller.clearCalls();

    control.step(DEADLINE_MS + 1);
    control.step(DEADLINE_MS + 2);

    TEST_ASSERT_EQUAL_STRING("", controller.calls.c_str());
}

void test_wheel_control_resumes_on_schedule_after_fresh_command()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    control.step(DEADLINE_MS);
    const uint32_t new_arrival_ms = DEADLINE_MS + 20;
    control.receiveCommand(TEST_COMMAND, new_arrival_ms);
    controller.clearCalls();

    control.step(DEADLINE_MS + TEST_CONTROL_PERIOD_MS);

    TEST_ASSERT_EQUAL_STRING("read write", controller.calls.c_str());
}

void test_wheel_control_waits_until_control_period_has_elapsed()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    const uint32_t last_step_ms = ARRIVAL_MS + TEST_CONTROL_PERIOD_MS;
    control.step(last_step_ms);
    controller.clearCalls();

    control.step(last_step_ms + TEST_CONTROL_PERIOD_MS - 1);

    TEST_ASSERT_EQUAL_STRING("", controller.calls.c_str());
}

void test_wheel_control_runs_one_step_after_delayed_loop()
{
    // The loop resumes after many missed control periods, 1 ms before the command expires.
    // Repeating step() at that time must not run another control step
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    control.step(ARRIVAL_MS + TEST_CONTROL_PERIOD_MS);
    controller.clearCalls();

    const uint32_t delayed_ms = DEADLINE_MS - 1;
    control.step(delayed_ms);
    control.step(delayed_ms);

    TEST_ASSERT_EQUAL_STRING("read write", controller.calls.c_str());
}

void test_wheel_control_preserves_schedule_when_commands_arrive()
{
    // A command 1 ms before the next control step doesn't move that step
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    control.receiveCommand(TEST_COMMAND, ARRIVAL_MS);
    const uint32_t last_step_ms = ARRIVAL_MS + TEST_CONTROL_PERIOD_MS;
    control.step(last_step_ms);
    const uint32_t next_step_ms = last_step_ms + TEST_CONTROL_PERIOD_MS;
    control.receiveCommand(TEST_COMMAND, next_step_ms - 1);
    controller.clearCalls();

    control.step(next_step_ms);

    TEST_ASSERT_EQUAL_STRING("read write", controller.calls.c_str());
}

void test_wheel_control_runs_on_schedule_across_clock_wrap()
{
    // The last control step 21 ms before millis() returns to zero. The sum for the next step
    // wraps like millis() does, to 29 ms
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    const uint32_t ms_before_wrap = 21;
    const uint32_t last_step_ms = UINT32_MAX - (ms_before_wrap - 1);
    control.receiveCommand(TEST_COMMAND, last_step_ms);
    control.step(last_step_ms);
    controller.clearCalls();

    const uint32_t next_step_ms = last_step_ms + TEST_CONTROL_PERIOD_MS;
    control.step(next_step_ms - 1);
    control.step(next_step_ms);

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
    RUN_TEST(test_wheel_control_stays_timed_out_after_complete_clock_cycle);
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
