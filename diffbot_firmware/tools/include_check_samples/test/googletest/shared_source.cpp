// A sample for the include check in the environment native_googletest, which the workflow
// Firmware checks. It lies in googletest but in no suite's folder, as a source that several
// suites share, and includes fakes/sample_fake.h from the folder test above it. pio test adds
// that folder for every test build, so the check passes only when it adds it too. These
// folders mirror diffbot_firmware/test. They aren't a test suite: PlatformIO doesn't build them.
#include "fakes/sample_fake.h"

static_assert(SAMPLE_FAKE_VALUE == 1, "the shared fake of the folder test");
