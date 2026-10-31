// Unit tests of CommandTimeout (lib/command_timeout/command_timeout.h) on the PC. Run them with:
//   pio test -e native
#include <unity.h>

#include <stdint.h>

#include "command_timeout.h"

void setUp() {}
void tearDown() {}

static const uint32_t TIMEOUT_MS = 1000;

void test_command_timeout_is_timed_out_until_the_first_command()
{
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    TEST_ASSERT_TRUE(timeout.timedOut());
    // No command yet: no event to log, the motors stay off
    TEST_ASSERT_FALSE(timeout.update(5000, 0, 0));
    TEST_ASSERT_TRUE(timeout.timedOut());
    // The first command arrives at 5000
    TEST_ASSERT_FALSE(timeout.update(5000, 5000, 1));
    TEST_ASSERT_FALSE(timeout.timedOut());
}

void test_command_timeout_fires_once_when_the_command_becomes_too_old()
{
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    TEST_ASSERT_FALSE(timeout.update(5000, 5000, 1));
    TEST_ASSERT_FALSE(timeout.update(5999, 5000, 1));
    TEST_ASSERT_TRUE(timeout.update(6000, 5000, 1));
    TEST_ASSERT_TRUE(timeout.timedOut());
    // The following passes don't fire again, so the motors are stopped and logged once
    TEST_ASSERT_FALSE(timeout.update(6001, 5000, 1));
    TEST_ASSERT_FALSE(timeout.update(9000, 5000, 1));
    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_command_timeout_ends_and_rearms_when_a_new_command_arrives()
{
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    timeout.update(5000, 5000, 1);
    TEST_ASSERT_TRUE(timeout.update(6000, 5000, 1));
    // A command arrives at 7000
    TEST_ASSERT_FALSE(timeout.update(7000, 7000, 2));
    TEST_ASSERT_FALSE(timeout.timedOut());
    // And stops again: the timeout fires once more
    TEST_ASSERT_TRUE(timeout.update(8000, 7000, 2));
    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_command_timeout_measures_the_age_across_the_millis_wrap()
{
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    // The last command 200 ms before millis() returns to zero
    const uint32_t last = 0xFFFFFFFFu - 199u;
    TEST_ASSERT_FALSE(timeout.update(last, last, 1));
    TEST_ASSERT_FALSE(timeout.update(500, last, 1));  // 700 ms later
    TEST_ASSERT_TRUE(timeout.update(800, last, 1));   // 1000 ms later
}

// Without a new command for a whole cycle of millis(), about 49.7 days, the old time looks new
// again. The timeout must stay on until a command really arrives.
void test_command_timeout_stays_timed_out_through_a_whole_millis_cycle_without_commands()
{
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    timeout.update(5000, 5000, 1);
    TEST_ASSERT_TRUE(timeout.update(6000, 5000, 1));
    const uint32_t a_cycle_later = 5000u;  // millis() went once around and is at 5000 again
    TEST_ASSERT_FALSE(timeout.update(a_cycle_later, 5000, 1));
    TEST_ASSERT_TRUE(timeout.timedOut());
    TEST_ASSERT_FALSE(timeout.update(a_cycle_later + 500, 5000, 1));
    TEST_ASSERT_TRUE(timeout.timedOut());
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_command_timeout_is_timed_out_until_the_first_command);
    RUN_TEST(test_command_timeout_fires_once_when_the_command_becomes_too_old);
    RUN_TEST(test_command_timeout_ends_and_rearms_when_a_new_command_arrives);
    RUN_TEST(test_command_timeout_measures_the_age_across_the_millis_wrap);
    RUN_TEST(test_command_timeout_stays_timed_out_through_a_whole_millis_cycle_without_commands);
    return UNITY_END();
}
