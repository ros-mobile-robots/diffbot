// A sample for the include check in the environment native_googletest, which the workflow
// Firmware checks. It lies in a sub folder of a suite's folder, as PlatformIO allows, and
// includes suite_options.h, which the suite's folder and the folder above it both have. The
// check passes only when it searches the suite's folder first, as pio test does. These folders
// mirror test/googletest. They aren't a test suite: PlatformIO doesn't build them.
#include "suite_options.h"

static_assert(SUITE_OPTIONS_FOLDER_LEVEL == 2, "the suite's own copy of suite_options.h");
