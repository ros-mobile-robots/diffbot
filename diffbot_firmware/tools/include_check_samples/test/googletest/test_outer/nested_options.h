// The copy of the header in the enclosing suite's folder for the include check's nested
// sample, test_inner/inner_case.cpp. pio test doesn't add an enclosing suite's folder for a
// nested suite, so a check that finds this copy fails on the error.
#error "The include check searched an enclosing suite's folder"
