// Unit tests of CommandTimeout (lib/command_timeout/command_timeout.h) on the PC. Run them with:
//   pio test -e native
#include <unity.h>

#include <stdint.h>

#include "command_timeout.h"

void setUp() {}
void tearDown() {}

static const uint32_t TIMEOUT_MS = 1000;

void test_no_timeout_while_commands_are_new()
{
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    TEST_ASSERT_FALSE(timeout.update(5000, 5000));
    TEST_ASSERT_FALSE(timeout.update(5999, 5000));
    TEST_ASSERT_FALSE(timeout.timedOut());
}

void test_fires_once_when_the_command_becomes_too_old()
{
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    TEST_ASSERT_TRUE(timeout.update(6000, 5000));
    TEST_ASSERT_TRUE(timeout.timedOut());
    // The following passes don't fire again, so the motors are stopped and logged once
    TEST_ASSERT_FALSE(timeout.update(6001, 5000));
    TEST_ASSERT_FALSE(timeout.update(9000, 5000));
    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_a_new_command_ends_the_timeout_and_rearms_it()
{
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    TEST_ASSERT_TRUE(timeout.update(6000, 5000));
    // A command arrives at 7000
    TEST_ASSERT_FALSE(timeout.update(7000, 7000));
    TEST_ASSERT_FALSE(timeout.timedOut());
    // And stops again: the timeout fires once more
    TEST_ASSERT_TRUE(timeout.update(8000, 7000));
    TEST_ASSERT_TRUE(timeout.timedOut());
}

void test_millis_returning_to_zero()
{
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    // The last command 200 ms before millis() returns to zero
    const uint32_t last = 0xFFFFFFFFu - 199u;
    TEST_ASSERT_FALSE(timeout.update(500, last));  // 700 ms later
    TEST_ASSERT_TRUE(timeout.update(800, last));   // 1000 ms later
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_no_timeout_while_commands_are_new);
    RUN_TEST(test_fires_once_when_the_command_becomes_too_old);
    RUN_TEST(test_a_new_command_ends_the_timeout_and_rearms_it);
    RUN_TEST(test_millis_returning_to_zero);
    return UNITY_END();
}
