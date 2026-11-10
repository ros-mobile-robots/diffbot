// Unit tests of the firmware's wheel PID (lib/pid) on the PC. Run them with:
//   pio test -e native
#include <unity.h>

#include "pid.h"

// Fixed inputs of these tests, independent of the robot's tuning (K_P, K_I, K_D and PWM_MAX in
// diffbot_base_config.h), so that a change of the tuning doesn't change what the tests check
constexpr float TEST_KP = 0.6f;
constexpr float TEST_KI = 0.3f;
constexpr float TEST_KD = 0.5f;
constexpr float TEST_OUTPUT_LIMIT = 255.0f;

// A wheel held back: its target is 10 rad/s, it turns at only 0.2 rad/s
constexpr float HELD_BACK_TARGET = 10.0f;
constexpr float HELD_BACK_VELOCITY = 0.2f;

// Constructs a PID with the fixed test gains and output limits
diffbot::PID makeTestPid()
{
    return diffbot::PID(-TEST_OUTPUT_LIMIT, TEST_OUTPUT_LIMIT, TEST_KP, TEST_KI, TEST_KD);
}

// 100 control steps with the wheel held back. Each step adds the raw error, 10 - 0.2 = 9.8, to
// integral(), which reaches about 980. That is neither the weighted I part of the output nor a
// time integral in seconds
void windUp(diffbot::PID& pid)
{
    for (int i = 0; i < 100; ++i)
        pid.compute(HELD_BACK_TARGET, HELD_BACK_VELOCITY);
}

void setUp() {}
void tearDown() {}

void test_pid_has_zero_state_when_constructed()
{
    diffbot::PID pid = makeTestPid();

    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.proportional());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.derivative());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.prev_error());
}

void test_pid_computes_first_output_without_previous_error()
{
    diffbot::PID pid = makeTestPid();

    const double output = pid.compute(1.0f, 0.0f);

    // An error of 1: P, I and D each see 1, D because the previous error is zero
    TEST_ASSERT_EQUAL_FLOAT(TEST_KP + TEST_KI + TEST_KD, output);
}

void test_pid_clamps_output_when_above_upper_limit()
{
    diffbot::PID pid = makeTestPid();

    const float far_above_target = 1000.0f; // P, I and D together far exceed the limit
    const double output = pid.compute(far_above_target, 0.0f);

    TEST_ASSERT_EQUAL_FLOAT(TEST_OUTPUT_LIMIT, output);
}

void test_pid_clamps_output_when_below_lower_limit()
{
    diffbot::PID pid = makeTestPid();

    const float far_below_target = -1000.0f; // P, I and D together far exceed the limit
    const double output = pid.compute(far_below_target, 0.0f);

    TEST_ASSERT_EQUAL_FLOAT(-TEST_OUTPUT_LIMIT, output);
}

// Why BaseController::eStop() resets the PIDs: with these test inputs, zeroing only the target
// leaves the computed output at the positive limit
void test_pid_keeps_positive_output_when_only_target_is_zeroed()
{
    diffbot::PID pid = makeTestPid();
    windUp(pid);

    const double output = pid.compute(0.0f, HELD_BACK_VELOCITY);

    TEST_ASSERT_EQUAL_FLOAT(TEST_OUTPUT_LIMIT, output);
}

void test_pid_clears_stored_state_when_reset()
{
    diffbot::PID pid = makeTestPid();
    windUp(pid);
    // A changed error, so that the derivative isn't zero either
    pid.compute(HELD_BACK_TARGET - 1.0f, HELD_BACK_VELOCITY);

    pid.reset();

    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.proportional());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.derivative());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.prev_error());
}

void test_pid_has_no_stale_positive_output_after_reset()
{
    diffbot::PID pid = makeTestPid();
    windUp(pid);
    pid.reset();

    const double output = pid.compute(0.0f, HELD_BACK_VELOCITY);

    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(0.0f, output);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_pid_has_zero_state_when_constructed);
    RUN_TEST(test_pid_computes_first_output_without_previous_error);
    RUN_TEST(test_pid_clamps_output_when_above_upper_limit);
    RUN_TEST(test_pid_clamps_output_when_below_lower_limit);
    RUN_TEST(test_pid_keeps_positive_output_when_only_target_is_zeroed);
    RUN_TEST(test_pid_clears_stored_state_when_reset);
    RUN_TEST(test_pid_has_no_stale_positive_output_after_reset);
    return UNITY_END();
}
