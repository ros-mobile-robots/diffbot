// A sample for the include check in the environment native_googletest, which the workflow
// Firmware checks. It lies in deep, a grouping folder below googletest, in no suite, as a
// source that the suites in deep share, and includes level_options.h, which googletest and deep
// both have. The angle brackets skip the file's own folder, which a quoted include searches
// first, so the include folders decide. A suite in deep gets googletest before deep, so the
// check passes only when it gives this file the grouping folders in that order too. These
// folders mirror diffbot_firmware/test. They aren't a test suite: PlatformIO doesn't build them.
#include <level_options.h>

static_assert(LEVEL_OPTIONS_FOLDER_DEPTH == 1, "the copy of level_options.h in googletest");
