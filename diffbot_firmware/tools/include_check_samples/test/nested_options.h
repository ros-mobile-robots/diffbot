// The copy of the header in the folder test for the include check's nested sample,
// googletest/test_outer/test_inner/inner_case.cpp. pio test adds this folder for every test
// build, so the sample finds this copy.
#ifndef DIFFBOT_INCLUDE_CHECK_SAMPLE_NESTED_OPTIONS_H
#define DIFFBOT_INCLUDE_CHECK_SAMPLE_NESTED_OPTIONS_H

// Where this copy lies: in the folder test
constexpr bool NESTED_OPTIONS_IN_TEST_FOLDER = true;

#endif  // DIFFBOT_INCLUDE_CHECK_SAMPLE_NESTED_OPTIONS_H
