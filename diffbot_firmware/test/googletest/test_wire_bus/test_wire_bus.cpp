// Unit tests of WireBusT (lib/imu/wire_bus.h) on the PC, with GoogleTest and a fake Wire library.
// Run them with:
//   pio test -e native_googletest
#include <gtest/gtest.h>

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <array>
#include <deque>
#include <numeric>
#include <ostream>
#include <string>
#include <vector>

#include "wire_bus.h"

// The transfers of these tests use a BNO055's values as the example: the sensor's I2C address,
// its mode register with the value for the IMU mode, and its first data register. WireBusT
// passes each of them to the Wire library unchanged
constexpr uint8_t DEVICE_ADDRESS = 0x28;
// The BNO055's other I2C address, for a breakout whose ADR pin is high (IMU_BNO055_ADDRESS in
// lib/config/diffbot_base_config.h)
constexpr uint8_t ALTERNATE_DEVICE_ADDRESS = 0x29;
constexpr uint8_t WRITTEN_REGISTER = 0x3D;
constexpr uint8_t WRITTEN_VALUE = 0x08;
constexpr uint8_t READ_REGISTER = 0x08;

// A result of Wire's endTransmission() that isn't 0, the value for success: no device
// acknowledged the address
constexpr uint8_t ADDRESS_NOT_ACKNOWLEDGED = 2;

// The two values of endTransmission()'s argument. A stop ends the transfer and frees the bus. A
// repeated start keeps the bus, so that the read follows the register write directly
constexpr bool WITH_STOP = true;
constexpr bool WITH_REPEATED_START = false;

// The first byte that FakeWire delivers to a read. Each further byte is one higher
constexpr uint8_t FIRST_DELIVERED_BYTE = 0x10;

// The length of the reads that get all their bytes: three bytes have a first, a middle and a
// last one
constexpr size_t READ_LENGTH_BYTES = 3;
// The length of a whole sample: the 18 data bytes that the BNO055 driver reads in one transfer
constexpr int SAMPLE_LENGTH_BYTES = 18;

// A byte as text, such as 0x28
std::string hexByte(uint8_t value)
{
    char text[sizeof("0xFF")];
    snprintf(text, sizeof(text), "0x%02X", value);
    return text;
}

// Each Wire call as text, such as beginTransmission(0x28). FakeWire records its calls with these
// functions, and the tests write the calls that they expect with the same functions
std::string beginTransmissionCall(uint8_t address)
{
    return "beginTransmission(" + hexByte(address) + ")";
}
std::string writeCall(uint8_t value) { return "write(" + hexByte(value) + ")"; }
std::string endTransmissionCall(bool stop)
{
    return std::string("endTransmission(") + (stop ? "true" : "false") + ")";
}
std::string requestFromCall(uint8_t address, uint8_t length)
{
    return "requestFrom(" + hexByte(address) + ", " + std::to_string(length) + ")";
}

// The calls of Arduino's TwoWire that WireBusT makes, with results a test can choose. FakeWire
// records the calls that start, fill and end a transfer, in their order. available() and read()
// only hand out the received bytes
class FakeWire
{
public:
    std::vector<std::string> calls;  ///< such as beginTransmission(0x28), write(0x3D)
    std::deque<uint8_t> received;
    uint8_t end_result = 0;        ///< endTransmission(): 0 is success
    int reported_count = -1;       ///< requestFrom()'s result; -1: the requested length
    int delivered_count = -1;      ///< bytes really received; -1: the requested length

    void beginTransmission(uint8_t address) { calls.push_back(beginTransmissionCall(address)); }
    size_t write(uint8_t value)
    {
        calls.push_back(writeCall(value));
        return 1;
    }
    uint8_t endTransmission(bool stop = true)
    {
        calls.push_back(endTransmissionCall(stop));
        return end_result;
    }
    uint8_t requestFrom(uint8_t address, uint8_t length)
    {
        calls.push_back(requestFromCall(address, length));
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

    const std::vector<std::string> expected_calls{
        beginTransmissionCall(DEVICE_ADDRESS), writeCall(WRITTEN_REGISTER),
        writeCall(WRITTEN_VALUE), endTransmissionCall(WITH_STOP)};
    EXPECT_EQ(wire_.calls, expected_calls);
}

TEST_F(WireBusTest, WriteRegisterFailsWhenTheAddressIsNotAcknowledged)
{
    wire_.end_result = ADDRESS_NOT_ACKNOWLEDGED;

    EXPECT_FALSE(bus_.writeRegister(DEVICE_ADDRESS, WRITTEN_REGISTER, WRITTEN_VALUE));
}

// The result of each write is its own: a successful write before doesn't hide a failure
TEST_F(WireBusTest, WriteRegisterFailsWhenTheAddressIsNotAcknowledgedAfterASuccessfulWrite)
{
    // 1. A write that the device acknowledges
    ASSERT_TRUE(bus_.writeRegister(DEVICE_ADDRESS, WRITTEN_REGISTER, WRITTEN_VALUE));

    // 2. The next write on the same bus, which no device acknowledges
    wire_.end_result = ADDRESS_NOT_ACKNOWLEDGED;
    EXPECT_FALSE(bus_.writeRegister(DEVICE_ADDRESS, WRITTEN_REGISTER, WRITTEN_VALUE));
}

TEST_F(WireBusTest, WriteRegisterSendsToTheAlternateAddress)
{
    EXPECT_TRUE(bus_.writeRegister(ALTERNATE_DEVICE_ADDRESS, WRITTEN_REGISTER, WRITTEN_VALUE));

    const std::vector<std::string> expected_calls{
        beginTransmissionCall(ALTERNATE_DEVICE_ADDRESS), writeCall(WRITTEN_REGISTER),
        writeCall(WRITTEN_VALUE), endTransmissionCall(WITH_STOP)};
    EXPECT_EQ(wire_.calls, expected_calls);
}

TEST_F(WireBusTest, ReadRegistersUsesARepeatedStart)
{
    std::array<uint8_t, READ_LENGTH_BYTES> data{};

    EXPECT_TRUE(bus_.readRegisters(DEVICE_ADDRESS, READ_REGISTER, data.data(), data.size()));

    // The register write ends with a repeated start, and the read request comes after it
    const std::vector<std::string> expected_calls{
        beginTransmissionCall(DEVICE_ADDRESS), writeCall(READ_REGISTER),
        endTransmissionCall(WITH_REPEATED_START),
        requestFromCall(DEVICE_ADDRESS, READ_LENGTH_BYTES)};
    EXPECT_EQ(wire_.calls, expected_calls);
    const std::array<uint8_t, READ_LENGTH_BYTES> delivered_bytes{
        FIRST_DELIVERED_BYTE, FIRST_DELIVERED_BYTE + 1, FIRST_DELIVERED_BYTE + 2};
    EXPECT_EQ(data, delivered_bytes);
}

TEST_F(WireBusTest, ReadRegistersReadsFromTheAlternateAddress)
{
    std::array<uint8_t, READ_LENGTH_BYTES> data{};

    EXPECT_TRUE(
        bus_.readRegisters(ALTERNATE_DEVICE_ADDRESS, READ_REGISTER, data.data(), data.size()));

    const std::vector<std::string> expected_calls{
        beginTransmissionCall(ALTERNATE_DEVICE_ADDRESS), writeCall(READ_REGISTER),
        endTransmissionCall(WITH_REPEATED_START),
        requestFromCall(ALTERNATE_DEVICE_ADDRESS, READ_LENGTH_BYTES)};
    EXPECT_EQ(wire_.calls, expected_calls);
}

TEST_F(WireBusTest, ReadRegistersReturnsAllBytesOfASample)
{
    std::array<uint8_t, SAMPLE_LENGTH_BYTES> data{};

    EXPECT_TRUE(bus_.readRegisters(DEVICE_ADDRESS, READ_REGISTER, data.data(), data.size()));

    // FIRST_DELIVERED_BYTE, and each further byte one higher
    std::array<uint8_t, SAMPLE_LENGTH_BYTES> delivered_bytes{};
    std::iota(delivered_bytes.begin(), delivered_bytes.end(), FIRST_DELIVERED_BYTE);
    EXPECT_EQ(data, delivered_bytes);
}

TEST_F(WireBusTest, ReadRegistersFailsWithoutAReadRequestWhenTheRegisterWriteFails)
{
    wire_.end_result = ADDRESS_NOT_ACKNOWLEDGED;
    std::array<uint8_t, READ_LENGTH_BYTES> data{};

    EXPECT_FALSE(bus_.readRegisters(DEVICE_ADDRESS, READ_REGISTER, data.data(), data.size()));

    // The register write, and no read request after it
    const std::vector<std::string> expected_calls{
        beginTransmissionCall(DEVICE_ADDRESS), writeCall(READ_REGISTER),
        endTransmissionCall(WITH_REPEATED_START)};
    EXPECT_EQ(wire_.calls, expected_calls);
}

// A count that the Teensy 4's Wire can report after a failed bus acquisition, with or without
// data
constexpr int COUNT_AFTER_FAILED_BUS_ACQUISITION = 4;

// One way in which a read comes back short of the requested SAMPLE_LENGTH_BYTES
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
    std::array<uint8_t, SAMPLE_LENGTH_BYTES> data{};

    EXPECT_FALSE(bus_.readRegisters(DEVICE_ADDRESS, READ_REGISTER, data.data(), data.size()));

    // The read request asked for the whole sample, so the bus came back short of it
    const std::vector<std::string> expected_calls{
        beginTransmissionCall(DEVICE_ADDRESS), writeCall(READ_REGISTER),
        endTransmissionCall(WITH_REPEATED_START),
        requestFromCall(DEVICE_ADDRESS, SAMPLE_LENGTH_BYTES)};
    EXPECT_EQ(wire_.calls, expected_calls);
    EXPECT_EQ(wire_.available(), 0) << "bytes of the short read are left in the Wire library";
}

INSTANTIATE_TEST_SUITE_P(
    ShortReads, WireBusShortReadTest,
    ::testing::Values(
        ShortRead{"ReportedShortAllBytesThere", COUNT_AFTER_FAILED_BUS_ACQUISITION,
                  SAMPLE_LENGTH_BYTES},
        ShortRead{"BothShort", COUNT_AFTER_FAILED_BUS_ACQUISITION,
                  COUNT_AFTER_FAILED_BUS_ACQUISITION},
        ShortRead{"ReportedInFullOneByteMissing", SAMPLE_LENGTH_BYTES, SAMPLE_LENGTH_BYTES - 1},
        ShortRead{"ReportedOneShortAllBytesThere", SAMPLE_LENGTH_BYTES - 1, SAMPLE_LENGTH_BYTES}),
    [](const ::testing::TestParamInfo<ShortRead>& info) { return std::string(info.param.name); });

// GoogleTest registers every test above by itself, so main() lists none of them. main() returns
// the result of RUN_ALL_TESTS(), which is 1 if anything failed. With that exit status, PlatformIO
// also fails the run for a failure outside a test's body, which its reading of the output misses
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
