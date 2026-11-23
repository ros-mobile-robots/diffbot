// The copy in deep of the header for the include check's sample
// test_two_levels/two_levels_case.cpp. PlatformIO searches googletest before this folder, so
// a check that finds this copy first fails on the error.
#error "The include check searched deep before googletest"
