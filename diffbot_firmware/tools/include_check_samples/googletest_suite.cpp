// A sample for the include check in the environment native_googletest, which the workflow
// Firmware checks. It includes GoogleMock and GoogleTest, and it stops with an error unless the
// check defines what PlatformIO defines when pio test compiles a suite. It isn't a test suite:
// PlatformIO doesn't build it.
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#if !defined(PIO_UNIT_TESTING) || !defined(UNIT_TEST) || !defined(PLATFORMIO) || PLATFORMIO < 60000
#error "The include check doesn't define what PlatformIO defines for a test build"
#endif

TEST(IncludeCheckSample, ComparesWithAGoogleMockMatcher)
{
    EXPECT_THAT(2, testing::Gt(1));
}
