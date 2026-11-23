// The shared copy of the header for the include check's order sample,
// test_include_order/cases/order_case.cpp. pio test searches the suite's folder before this
// one, so the sample finds the suite's copy. A check that finds this copy fails on the error.
#error "The include check searched test/googletest before the suite's folder"
