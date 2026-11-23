// A sample for the include check in the environment native_googletest, which the workflow
// Firmware checks. Its suite lies two folders below test, in deep, and it includes
// level_options.h, which googletest and deep both have. PlatformIO searches the folders above
// a suite with the outermost first, so the check passes only when it finds the copy in
// googletest. These folders mirror diffbot_firmware/test. They aren't a test suite:
// PlatformIO doesn't build them.
#include "level_options.h"

static_assert(LEVEL_OPTIONS_FOLDER_DEPTH == 1, "the copy of level_options.h in googletest");
