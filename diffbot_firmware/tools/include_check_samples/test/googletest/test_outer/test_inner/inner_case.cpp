// A sample for the include check in the environment native_googletest, which the workflow
// Firmware checks. It lies in a suite inside another suite's folder and includes
// nested_options.h, which the enclosing suite's folder and the folder test both have. pio test
// stops at the enclosing suite's folder and adds test, so the check passes only when it finds
// the copy in test. These folders mirror diffbot_firmware/test. They aren't a test suite:
// PlatformIO doesn't build them.
#include "nested_options.h"

static_assert(NESTED_OPTIONS_IN_TEST_FOLDER, "the copy of nested_options.h in the folder test");
