// Unit tests of CommandTimeout (lib/command_timeout/command_timeout.h) on the PC. Run them with:
//   pio test -e native
#include <unity.h>

#include <stdint.h>

#include "command_timeout.h"

// A fixed timeout for these tests, independent of the robot's COMMAND_TIMEOUT_MS, so that a change
// of the setting doesn't change what the tests check
constexpr uint32_t TEST_TIMEOUT_MS = 1000;

// When a test's command arrives, and the first time at which that command is too old
constexpr uint32_t ARRIVAL_MS = 100;
constexpr uint32_t DEADLINE_MS = ARRIVAL_MS + TEST_TIMEOUT_MS;

void setUp() {}
void tearDown() {}

void test_command_timeout_is_timed_out_when_constructed()
{
    const diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);

    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_command_timeout_stays_timed_out_without_command()
{
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);

    timeout.update(5 * TEST_TIMEOUT_MS);

    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_command_timeout_is_fresh_after_command()
{
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);

    timeout.commandArrived(ARRIVAL_MS);

    TEST_ASSERT_FALSE(timeout.timedOut());
}

void test_command_timeout_stays_fresh_before_deadline()
{
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    timeout.commandArrived(ARRIVAL_MS);

    timeout.update(DEADLINE_MS - 1);

    TEST_ASSERT_FALSE(timeout.timedOut());
}

void test_command_timeout_expires_at_deadline()
{
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    timeout.commandArrived(ARRIVAL_MS);

    timeout.update(DEADLINE_MS);

    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_command_timeout_restarts_on_new_command()
{
    // A second command halfway to the first one's deadline moves the deadline
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    timeout.commandArrived(ARRIVAL_MS);
    timeout.commandArrived(ARRIVAL_MS + TEST_TIMEOUT_MS / 2);

    timeout.update(DEADLINE_MS);

    TEST_ASSERT_FALSE(timeout.timedOut());
}

void test_command_timeout_restarts_on_command_with_same_timestamp()
{
    // After a whole cycle of millis(), about 49.7 days, a new command arrives at the same clock
    // value as the old one
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    timeout.commandArrived(ARRIVAL_MS);
    timeout.update(DEADLINE_MS);

    timeout.commandArrived(ARRIVAL_MS);
    timeout.update(ARRIVAL_MS);

    TEST_ASSERT_FALSE(timeout.timedOut());
}

void test_command_timeout_expires_across_clock_wrap()
{
    // The command arrives 200 ms before millis() returns to zero. Its deadline wraps like
    // millis() does, to 800 ms
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    const uint32_t ms_before_wrap = 200;
    const uint32_t arrival_ms = UINT32_MAX - (ms_before_wrap - 1);
    timeout.commandArrived(arrival_ms);

    timeout.update(arrival_ms + TEST_TIMEOUT_MS);

    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_command_timeout_stays_timed_out_after_complete_clock_cycle()
{
    // A whole cycle of millis() later, without a new command, the clock shows the old command's
    // arrival time again. That reading alone must not make the old command new
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    timeout.commandArrived(ARRIVAL_MS);
    timeout.update(DEADLINE_MS);

    timeout.update(ARRIVAL_MS);

    TEST_ASSERT_TRUE(timeout.timedOut());
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_command_timeout_is_timed_out_when_constructed);
    RUN_TEST(test_command_timeout_stays_timed_out_without_command);
    RUN_TEST(test_command_timeout_is_fresh_after_command);
    RUN_TEST(test_command_timeout_stays_fresh_before_deadline);
    RUN_TEST(test_command_timeout_expires_at_deadline);
    RUN_TEST(test_command_timeout_restarts_on_new_command);
    RUN_TEST(test_command_timeout_restarts_on_command_with_same_timestamp);
    RUN_TEST(test_command_timeout_expires_across_clock_wrap);
    RUN_TEST(test_command_timeout_stays_timed_out_after_complete_clock_cycle);
    return UNITY_END();
}
