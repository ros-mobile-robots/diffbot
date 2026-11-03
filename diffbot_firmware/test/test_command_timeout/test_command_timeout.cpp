// Unit tests of CommandTimeout (lib/command_timeout/command_timeout.h) on the PC. Run them with:
//   pio test -e native
#include <unity.h>

#include <stdint.h>

#include "command_timeout.h"

// A fixed input of these tests, independent of the robot's COMMAND_TIMEOUT_MS. The times in the
// tests are written out for it
constexpr uint32_t TEST_TIMEOUT_MS = 1000;

void setUp() {}
void tearDown() {}

void test_command_timeout_is_inhibited_when_constructed()
{
    const diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);

    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_command_timeout_stays_inhibited_without_command()
{
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);

    timeout.update(5000);

    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_command_timeout_arms_on_command()
{
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);

    timeout.commandArrived(100);

    TEST_ASSERT_FALSE(timeout.timedOut());
}

void test_command_timeout_stays_fresh_before_deadline()
{
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    timeout.commandArrived(100);

    timeout.update(1099);

    TEST_ASSERT_FALSE(timeout.timedOut());
}

void test_command_timeout_expires_at_deadline()
{
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    timeout.commandArrived(100);

    timeout.update(1100);

    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_command_timeout_refreshes_deadline_on_new_command()
{
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    timeout.commandArrived(100);
    timeout.commandArrived(600);

    timeout.update(1100);

    TEST_ASSERT_FALSE(timeout.timedOut());
}

void test_command_timeout_rearms_on_same_timestamp_receipt()
{
    // After a whole cycle of millis(), about 49.7 days, a new command arrives at the same time
    // as the old one
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    timeout.commandArrived(100);
    timeout.update(1100);

    timeout.commandArrived(100);
    timeout.update(100);

    TEST_ASSERT_FALSE(timeout.timedOut());
}

void test_command_timeout_expires_across_clock_wrap()
{
    // The command arrives 200 ms before millis() returns to zero
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    timeout.commandArrived(UINT32_MAX - 199);

    timeout.update(800);

    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_command_timeout_stays_latched_after_complete_clock_cycle()
{
    // A whole cycle of millis() later, without a new command: the old command's time looks new
    diffbot::CommandTimeout timeout(TEST_TIMEOUT_MS);
    timeout.commandArrived(100);
    timeout.update(1100);

    timeout.update(100);

    TEST_ASSERT_TRUE(timeout.timedOut());
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_command_timeout_is_inhibited_when_constructed);
    RUN_TEST(test_command_timeout_stays_inhibited_without_command);
    RUN_TEST(test_command_timeout_arms_on_command);
    RUN_TEST(test_command_timeout_stays_fresh_before_deadline);
    RUN_TEST(test_command_timeout_expires_at_deadline);
    RUN_TEST(test_command_timeout_refreshes_deadline_on_new_command);
    RUN_TEST(test_command_timeout_rearms_on_same_timestamp_receipt);
    RUN_TEST(test_command_timeout_expires_across_clock_wrap);
    RUN_TEST(test_command_timeout_stays_latched_after_complete_clock_cycle);
    return UNITY_END();
}
