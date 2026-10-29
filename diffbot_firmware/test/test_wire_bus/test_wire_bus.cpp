// Unit tests of WireBusT (lib/imu/wire_bus.h) on the PC, with a fake Wire library. Run them with:
//   pio test -e native
#include <unity.h>

#include <stddef.h>
#include <stdint.h>

#include <deque>
#include <vector>

#include "wire_bus.h"

// The calls of Arduino's TwoWire that WireBusT makes, with results a test can choose
class FakeWire
{
public:
    std::vector<uint8_t> sent;
    std::deque<uint8_t> received;
    uint8_t end_result = 0;        ///< endTransmission(): 0 is success
    int reported_count = -1;       ///< requestFrom()'s result; -1: the requested length
    int delivered_count = -1;      ///< bytes really received; -1: the requested length
    bool stop_after_write = true;  ///< the last endTransmission's argument

    void beginTransmission(uint8_t) { sent.clear(); }
    size_t write(uint8_t value)
    {
        sent.push_back(value);
        return 1;
    }
    uint8_t endTransmission(bool stop = true)
    {
        stop_after_write = stop;
        return end_result;
    }
    uint8_t requestFrom(uint8_t, uint8_t length)
    {
        received.clear();
        int n = delivered_count < 0 ? length : delivered_count;
        for (int i = 0; i < n; ++i)
            received.push_back((uint8_t)(0x10 + i));
        return (uint8_t)(reported_count < 0 ? length : reported_count);
    }
    int available() { return (int)received.size(); }
    int read()
    {
        if (received.empty())
            return -1;
        int value = received.front();
        received.pop_front();
        return value;
    }
};

void setUp() {}
void tearDown() {}

void test_write_register()
{
    FakeWire wire;
    diffbot::WireBusT<FakeWire> bus(wire);
    TEST_ASSERT_TRUE(bus.writeRegister(0x28, 0x3D, 0x08));
    TEST_ASSERT_EQUAL(2, wire.sent.size());
    TEST_ASSERT_EQUAL_HEX8(0x3D, wire.sent[0]);
    TEST_ASSERT_EQUAL_HEX8(0x08, wire.sent[1]);
    wire.end_result = 2; // address not acknowledged
    TEST_ASSERT_FALSE(bus.writeRegister(0x28, 0x3D, 0x08));
}

void test_read_registers_with_a_repeated_start()
{
    FakeWire wire;
    diffbot::WireBusT<FakeWire> bus(wire);
    uint8_t data[3] = {0};
    TEST_ASSERT_TRUE(bus.readRegisters(0x28, 0x08, data, 3));
    TEST_ASSERT_FALSE(wire.stop_after_write);
    TEST_ASSERT_EQUAL_HEX8(0x08, wire.sent[0]);
    TEST_ASSERT_EQUAL_HEX8(0x10, data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x12, data[2]);
}

void test_read_fails_when_the_register_write_fails()
{
    FakeWire wire;
    wire.end_result = 2;
    diffbot::WireBusT<FakeWire> bus(wire);
    uint8_t data[3] = {0};
    TEST_ASSERT_FALSE(bus.readRegisters(0x28, 0x08, data, 3));
}

void test_short_reads_fail_and_leave_nothing_behind()
{
    // The count requestFrom() reports and the bytes really received both have to be the length:
    // the Teensy 4's Wire can report 4 on a failed bus acquisition, with or without data
    const int cases[][2] = {
        {4, 18},  // reported short, all bytes there
        {4, 4},   // both short
        {18, 17}, // reported in full, one byte missing
        {17, 18}, // reported one short, all bytes there
    };
    uint8_t data[18] = {0};
    for (const auto& c : cases)
    {
        FakeWire wire;
        wire.reported_count = c[0];
        wire.delivered_count = c[1];
        diffbot::WireBusT<FakeWire> bus(wire);
        TEST_ASSERT_FALSE(bus.readRegisters(0x28, 0x08, data, 18));
        TEST_ASSERT_EQUAL(0, wire.available());
    }
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_write_register);
    RUN_TEST(test_read_registers_with_a_repeated_start);
    RUN_TEST(test_read_fails_when_the_register_write_fails);
    RUN_TEST(test_short_reads_fail_and_leave_nothing_behind);
    return UNITY_END();
}
