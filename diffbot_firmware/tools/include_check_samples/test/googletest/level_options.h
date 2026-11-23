// The copy in googletest of the header for the include check's sample
// deep/test_two_levels/two_levels_case.cpp. PlatformIO searches the folders above a suite with
// the outermost first, so the sample finds this copy before the one in deep.
#ifndef DIFFBOT_INCLUDE_CHECK_SAMPLE_LEVEL_OPTIONS_H
#define DIFFBOT_INCLUDE_CHECK_SAMPLE_LEVEL_OPTIONS_H

// Where this copy lies: one folder below test
constexpr int LEVEL_OPTIONS_FOLDER_DEPTH = 1;

#endif  // DIFFBOT_INCLUDE_CHECK_SAMPLE_LEVEL_OPTIONS_H
