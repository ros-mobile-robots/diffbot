// Unit tests of WireBusT (lib/imu/wire_bus.h) on the PC, with GoogleTest and a fake Wire library.
// Run them with:
//   pio test -e native_googletest
#include <gtest/gtest.h>

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <deque>
#include <ostream>
#include <string>
#include <vector>

#include "wire_bus.h"

// The transfers of these tests use a BNO055's values as the example: the sensor's I2C address,
// its mode register with the value for the IMU mode, and its first data register. WireBusT
// passes each of them to the Wire library unchanged
constexpr uint8_t DEVICE_ADDRESS = 0x28;
constexpr uint8_t WRITTEN_REGISTER = 0x3D;
constexpr uint8_t WRITTEN_VALUE = 0x08;
constexpr uint8_t READ_REGISTER = 0x08;

// A result of Wire's endTransmission() that isn't 0, the value for success: no device
// acknowledged the address
constexpr uint8_t ADDRESS_NOT_ACKNOWLEDGED = 2;

// The first byte that FakeWire delivers to a read. Each further byte is one higher
constexpr uint8_t FIRST_DELIVERED_BYTE = 0x10;

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
            received.push_back((uint8_t)(FIRST_DELIVERED_BYTE + i));
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

// A fake Wire library and a WireBusT on it. GoogleTest constructs the fixture anew for every
// test, so no test sees what another one sent or received
class WireBusTest : public ::testing::Test
{
protected:
    FakeWire wire_;
    diffbot::WireBusT<FakeWire> bus_{wire_};
};

TEST_F(WireBusTest, WriteRegisterSendsRegisterAndValue)
{
    EXPECT_TRUE(bus_.writeRegister(DEVICE_ADDRESS, WRITTEN_REGISTER, WRITTEN_VALUE));

    EXPECT_EQ(wire_.sent, (std::vector<uint8_t>{WRITTEN_REGISTER, WRITTEN_VALUE}));
}

TEST_F(WireBusTest, WriteRegisterFailsWhenTheAddressIsNotAcknowledged)
{
    wire_.end_result = ADDRESS_NOT_ACKNOWLEDGED;

    EXPECT_FALSE(bus_.writeRegister(DEVICE_ADDRESS, WRITTEN_REGISTER, WRITTEN_VALUE));
}

TEST_F(WireBusTest, ReadRegistersUsesARepeatedStart)
{
    std::array<uint8_t, 3> data{};

    EXPECT_TRUE(bus_.readRegisters(DEVICE_ADDRESS, READ_REGISTER, data.data(), data.size()));

    EXPECT_FALSE(wire_.stop_after_write) << "the register write ended with a stop";
    EXPECT_EQ(wire_.sent, (std::vector<uint8_t>{READ_REGISTER}));
    const std::array<uint8_t, 3> delivered_bytes{
        FIRST_DELIVERED_BYTE, FIRST_DELIVERED_BYTE + 1, FIRST_DELIVERED_BYTE + 2};
    EXPECT_EQ(data, delivered_bytes);
}

TEST_F(WireBusTest, ReadRegistersFailsWhenTheRegisterWriteFails)
{
    wire_.end_result = ADDRESS_NOT_ACKNOWLEDGED;
    std::array<uint8_t, 3> data{};

    EXPECT_FALSE(bus_.readRegisters(DEVICE_ADDRESS, READ_REGISTER, data.data(), data.size()));
}

// The length of the short reads' requests: the 18 data bytes of a BNO055 sample
constexpr int SAMPLE_LENGTH = 18;
// A count that the Teensy 4's Wire can report after a failed bus acquisition, with or without
// data
constexpr int COUNT_AFTER_FAILED_BUS_ACQUISITION = 4;

// One way in which a read comes back short of the requested SAMPLE_LENGTH bytes
struct ShortRead
{
    const char* name;     ///< The case's name in the test's output
    int reported_count;   ///< requestFrom()'s result
    int delivered_count;  ///< The bytes that really arrive
};

// GoogleTest prints a failed test's parameter with this operator
std::ostream& operator<<(std::ostream& output, const ShortRead& short_read)
{
    return output << "requestFrom() reports " << short_read.reported_count << " bytes, "
                  << short_read.delivered_count << " bytes arrive";
}

// The fixture of the parameterized test: WireBusTest's members, and one ShortRead per run
class WireBusShortReadTest : public WireBusTest, public ::testing::WithParamInterface<ShortRead>
{
};

// The count that requestFrom() reports and the bytes really received both have to be the
// requested length. GoogleTest runs this test once for each ShortRead below
TEST_P(WireBusShortReadTest, ReadRegistersFailsAndDrainsTheBytes)
{
    const ShortRead& short_read = GetParam();
    wire_.reported_count = short_read.reported_count;
    wire_.delivered_count = short_read.delivered_count;
    std::array<uint8_t, SAMPLE_LENGTH> data{};

    EXPECT_FALSE(bus_.readRegisters(DEVICE_ADDRESS, READ_REGISTER, data.data(), data.size()));

    EXPECT_EQ(wire_.available(), 0) << "bytes of the short read are left in the Wire library";
}

INSTANTIATE_TEST_SUITE_P(
    ShortReads, WireBusShortReadTest,
    ::testing::Values(
        ShortRead{"ReportedShortAllBytesThere", COUNT_AFTER_FAILED_BUS_ACQUISITION, SAMPLE_LENGTH},
        ShortRead{"BothShort", COUNT_AFTER_FAILED_BUS_ACQUISITION,
                  COUNT_AFTER_FAILED_BUS_ACQUISITION},
        ShortRead{"ReportedInFullOneByteMissing", SAMPLE_LENGTH, SAMPLE_LENGTH - 1},
        ShortRead{"ReportedOneShortAllBytesThere", SAMPLE_LENGTH - 1, SAMPLE_LENGTH}),
    [](const ::testing::TestParamInfo<ShortRead>& info) { return std::string(info.param.name); });

// GoogleTest registers every test above by itself, so main() lists none of them. main() returns
// the result of RUN_ALL_TESTS(), which is 1 if anything failed. With that exit status, PlatformIO
// also fails the run for a failure outside a test's body, which its reading of the output misses
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
