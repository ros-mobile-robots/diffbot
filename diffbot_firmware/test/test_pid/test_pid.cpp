// Unit tests of the firmware's wheel PID (lib/pid) on the PC. Run them with:
//   pio test -e native
#include <unity.h>

#include "pid.h"

void setUp() {}
void tearDown() {}

// The firmware's gains and limits (diffbot_base_config.h), written out here
static const float KP = 0.6f, KI = 0.3f, KD = 0.5f;
static const float OUT_MAX = 255.0f;

void test_starts_without_stored_error()
{
    diffbot::PID pid(-OUT_MAX, OUT_MAX, KP, KI, KD);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)pid.integral());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)pid.prev_error());
    // A first error of 1: P, I and D each see 1
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, KP + KI + KD, (float)pid.compute(1.0f, 0.0f));
}

void test_output_is_limited()
{
    diffbot::PID pid(-OUT_MAX, OUT_MAX, KP, KI, KD);
    TEST_ASSERT_EQUAL_FLOAT(OUT_MAX, (float)pid.compute(1000.0f, 0.0f));
    diffbot::PID pid2(-OUT_MAX, OUT_MAX, KP, KI, KD);
    TEST_ASSERT_EQUAL_FLOAT(-OUT_MAX, (float)pid2.compute(-1000.0f, 0.0f));
}

// The case of #146: a wheel held back winds the integral up, so a target of zero alone keeps
// the output at full power. After reset(), a target of zero with the wheel still turning gives
// no positive output.
void test_reset_clears_a_wound_up_integral()
{
    diffbot::PID pid(-OUT_MAX, OUT_MAX, KP, KI, KD);
    for (int i = 0; i < 100; ++i)
    {
        pid.compute(10.0f, 0.2f);  // the wheel barely turns
    }
    TEST_ASSERT_TRUE(pid.integral() > 900.0);
    TEST_ASSERT_EQUAL_FLOAT(OUT_MAX, (float)pid.compute(0.0f, 0.2f));  // without a reset: still full power

    pid.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)pid.integral());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)pid.prev_error());
    TEST_ASSERT_TRUE(pid.compute(0.0f, 0.2f) <= 0.0);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_without_stored_error);
    RUN_TEST(test_output_is_limited);
    RUN_TEST(test_reset_clears_a_wound_up_integral);
    return UNITY_END();
}
