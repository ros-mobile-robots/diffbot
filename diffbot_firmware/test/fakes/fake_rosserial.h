// A serial port and a clock for rosserial's node handle on the PC, for the native unit tests.
// rosserial's parser (ros/node_handle.h of the Rosserial Arduino Library) runs unchanged: the
// tests feed it the frames the host would send and read what it writes back.
#ifndef DIFFBOT_FAKE_ROSSERIAL_H
#define DIFFBOT_FAKE_ROSSERIAL_H

#include <stdint.h>

#include <deque>
#include <initializer_list>
#include <vector>

#include <ros/node_handle.h>
#include <rosserial_msgs/TopicInfo.h>

// The Teensy's millis() and micros(), set by the tests
struct FakeClock
{
    static uint64_t us;
    static uint32_t millis() { return (uint32_t)(us / 1000); }
    static uint32_t micros() { return (uint32_t)us; }
    static void setMs(uint32_t ms) { us = (uint64_t)ms * 1000; }
};

class FakeSerial
{
public:
    std::deque<uint8_t> input;                ///< Bytes from the host, not read yet
    std::vector<std::vector<uint8_t>> output; ///< Frames written to the host

    void init() {}
    uint32_t time() { return FakeClock::millis(); }
    int read()
    {
        if (input.empty())
            return -1;
        int byte = input.front();
        input.pop_front();
        return byte;
    }
    void write(uint8_t* data, int length) { output.push_back(std::vector<uint8_t>(data, data + length)); }

    /// Queues one frame from the host, in rosserial's format (protocol version 2)
    void receive(uint16_t topic, const std::vector<uint8_t>& data = std::vector<uint8_t>())
    {
        uint8_t length_low = data.size() & 0xFF;
        uint8_t length_high = (data.size() >> 8) & 0xFF;
        uint8_t topic_low = topic & 0xFF;
        uint8_t topic_high = (topic >> 8) & 0xFF;
        uint8_t frame[] = {0xFF, 0xFE, length_low, length_high, (uint8_t)(255 - ((length_low + length_high) % 256)),
                           topic_low, topic_high};
        input.insert(input.end(), frame, frame + sizeof(frame));
        unsigned sum = topic_low + topic_high;
        for (uint8_t byte : data)
        {
            input.push_back(byte);
            sum += byte;
        }
        input.push_back((uint8_t)(255 - (sum % 256)));
    }

    /// The host asks for the topics (a new connection)
    void negotiation() { receive(rosserial_msgs::TopicInfo::ID_PUBLISHER); }

    /// The host's answer to a time request: its ROS time
    void timeReply(uint32_t sec, uint32_t nsec = 0)
    {
        std::vector<uint8_t> data;
        for (uint32_t value : {sec, nsec})
            for (int i = 0; i < 4; ++i)
                data.push_back((value >> (8 * i)) & 0xFF);
        receive(rosserial_msgs::TopicInfo::ID_TIME, data);
    }

    /// Frames written on the firmware's own topics (ids from 100), not rosserial's
    int countTopics() const
    {
        int n = 0;
        for (const auto& frame : output)
            if (frame.size() > 6 && (frame[5] | (frame[6] << 8)) >= 100)
                ++n;
        return n;
    }

    /// Frames written for one topic id
    int count(uint16_t topic) const
    {
        int n = 0;
        for (const auto& frame : output)
            if (frame.size() > 6 && (frame[5] | (frame[6] << 8)) == topic)
                ++n;
        return n;
    }
};

#endif // DIFFBOT_FAKE_ROSSERIAL_H
