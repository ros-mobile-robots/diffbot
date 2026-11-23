// A sample for the include check in the environment native_googletest, which the workflow
// Firmware checks. It lies in a folder named test inside the suite test_include_order and
// includes suite_options.h from the suite's folder. Its suite is still test_include_order, and
// the test folder is still the first folder named test in its path, so the check passes only
// when it finds both that way. These folders mirror diffbot_firmware/test. They aren't a test
// suite: PlatformIO doesn't build them.
#include "suite_options.h"

static_assert(SUITE_OPTIONS_FOLDER_LEVEL == 2, "the suite's own copy of suite_options.h");
