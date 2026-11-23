// A sample for the include check, which the workflow Firmware checks. rosserial belongs to the
// environment native, where the check has to pass on this file, and not to native_googletest,
// where the check has to fail on it. It isn't a test suite: PlatformIO doesn't build it.
#include <ros/node_handle.h>

// A name from the header, so that the include is used
using SampleNodeHandleBase = ros::NodeHandleBase_;
