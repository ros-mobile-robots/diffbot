// A sample for the include check in the environment native_googletest, which the workflow
// Firmware checks. rosserial belongs to the environment native, so the check has to fail on
// this file. It isn't a test suite: PlatformIO doesn't build it.
#include <ros/node_handle.h>
