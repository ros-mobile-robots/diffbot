// Unit tests of RosWheelCommandSubscriber (lib/ros_wheel_commands/ros_wheel_command_subscriber.h)
// on the PC: rosserial's real parser with a fake serial port, and in the last tests a WheelControl
// with a fake controller behind the subscriber (test/fakes). Run them with:
//   pio test -e native
#include <unity.h>

#include <stdint.h>

#include <limits>
#include <vector>

#include <diffbot_msgs/WheelsCmdStamped.h>
#include <ros/node_handle.h>

#include "../fakes/fake_base_controller.h"
#include "../fakes/fake_rosserial.h"
#include "ros_wheel_command_subscriber.h"
#include "wheel_command.h"
#include "wheel_control.h"

// rosserial's own timeouts aren't under test: its clock (FakeSerial::time()) stays at zero
uint64_t FakeClock::us = 0;

typedef ros::NodeHandle_<FakeSerial> NodeHandle;

// rosserial gives the subscribers topic IDs from 100 on, in the order they subscribe. The tests
// send their messages with this ID, as serial_node.py on the robot's computer does
constexpr uint16_t WHEEL_COMMAND_TOPIC_ID = 100;

// The header stamp of the sent messages: a ROS time far from the tests' arrival times, so that a
// test fails if the subscriber takes the stamp as the arrival time
constexpr uint32_t HOST_STAMP_SEC = 1700000000;

// The wheel speeds that the tests send, and when the Teensy reads them
constexpr float LEFT_RAD_S = 1.25f;
constexpr float RIGHT_RAD_S = -2.5f;
constexpr uint32_t ARRIVAL_MS = 100;

// The timing of the last two tests, which put a real WheelControl behind the subscriber. Fixed
// values, independent of the robot's settings in diffbot_base_config.h
constexpr uint32_t TEST_TIMEOUT_MS = 1000;
constexpr uint32_t TEST_CONTROL_PERIOD_MS = 50;
constexpr diffbot::WheelControlConfig TEST_CONFIG = {TEST_TIMEOUT_MS, TEST_CONTROL_PERIOD_MS};

typedef diffbot::WheelControl<FakeBaseController, FakeTimeoutLog> Control;

// The subscriber's clock (Clock::millis()). sendCommand() sets it to each command's arrival time
struct TestClock
{
    uint32_t now_ms = 0;
    uint32_t millis() const { return now_ms; }
};

// One call of receiveCommand(): the command and its arrival time
struct ReceivedCommand
{
    diffbot::WheelCommand command;
    uint32_t received_ms;
};

// Records each receiveCommand() call of the subscriber
struct FakeReceiver
{
    std::vector<ReceivedCommand> received;

    void receiveCommand(diffbot::WheelCommand command, uint32_t received_ms)
    {
        received.push_back({command, received_ms});
    }
};

// One test's subscriber with what it needs: a node handle that uses rosserial's real parser on a
// fake serial port (FakeSerial), and the subscriber's clock, which the test controls. The
// subscriber registers before initNode(), as setup() on the robot does. sendCommand() puts the
// commands straight into the fake port. The topic negotiation with the computer isn't part of
// these tests.
template <class Receiver>
struct SubscriberTestFixture
{
    NodeHandle nh;
    TestClock clock;
    diffbot::RosWheelCommandSubscriber<Receiver, TestClock> subscriber;

    explicit SubscriberTestFixture(Receiver& receiver) : subscriber(receiver, clock)
    {
        subscriber.subscribe(nh);
        nh.initNode();
    }

    /// Plays the robot's computer: sends one wheel command message that holds wheel_speeds_rad_s
    /// (left and right, or a wrong number of values) and arrives at arrival_ms. The message's
    /// bytes go into the fake serial port, then spinOnce() runs as in loop(). spinOnce() parses
    /// the bytes with rosserial's real parser and calls the subscriber's callback
    void sendCommand(std::vector<float> wheel_speeds_rad_s, uint32_t arrival_ms)
    {
        diffbot_msgs::WheelsCmdStamped message;
        message.header.stamp.sec = HOST_STAMP_SEC;
        message.wheels_cmd.angular_velocities.joint_length =
            static_cast<uint32_t>(wheel_speeds_rad_s.size());
        message.wheels_cmd.angular_velocities.joint = wheel_speeds_rad_s.data();
        // The header and up to three values need less than 64 bytes
        std::vector<uint8_t> data(64);
        data.resize(message.serialize(data.data()));

        clock.now_ms = arrival_ms;
        // getHardware() returns the node handle's serial port, here the FakeSerial
        nh.getHardware()->queueFromHost(WHEEL_COMMAND_TOPIC_ID, data);
        nh.spinOnce();
    }
};

void setUp() {}
void tearDown() {}

void test_ros_wheel_command_subscriber_passes_command_with_teensy_arrival_time()
{
    FakeReceiver receiver;
    SubscriberTestFixture<FakeReceiver> fixture(receiver);

    fixture.sendCommand({LEFT_RAD_S, RIGHT_RAD_S}, ARRIVAL_MS);

    TEST_ASSERT_EQUAL_UINT(1, receiver.received.size());
    TEST_ASSERT_EQUAL_FLOAT(LEFT_RAD_S, receiver.received[0].command.left_rad_s);
    TEST_ASSERT_EQUAL_FLOAT(RIGHT_RAD_S, receiver.received[0].command.right_rad_s);
    TEST_ASSERT_EQUAL_UINT32(ARRIVAL_MS, receiver.received[0].received_ms);
}

void test_ros_wheel_command_subscriber_passes_every_command_in_same_millisecond()
{
    FakeReceiver receiver;
    SubscriberTestFixture<FakeReceiver> fixture(receiver);

    fixture.sendCommand({0.0f, 0.0f}, ARRIVAL_MS);
    fixture.sendCommand({LEFT_RAD_S, RIGHT_RAD_S}, ARRIVAL_MS);

    TEST_ASSERT_EQUAL_UINT(2, receiver.received.size());
    TEST_ASSERT_EQUAL_FLOAT(LEFT_RAD_S, receiver.received[1].command.left_rad_s);
    TEST_ASSERT_EQUAL_UINT32(ARRIVAL_MS, receiver.received[1].received_ms);
}

void test_ros_wheel_command_subscriber_reaches_own_receiver_with_two_instances()
{
    FakeReceiver first_receiver;
    FakeReceiver second_receiver;
    SubscriberTestFixture<FakeReceiver> first(first_receiver);
    SubscriberTestFixture<FakeReceiver> second(second_receiver);

    second.sendCommand({LEFT_RAD_S, RIGHT_RAD_S}, ARRIVAL_MS);

    TEST_ASSERT_EQUAL_UINT(0, first_receiver.received.size());
    TEST_ASSERT_EQUAL_UINT(1, second_receiver.received.size());
}

void test_ros_wheel_command_subscriber_drops_message_with_one_value()
{
    FakeReceiver receiver;
    SubscriberTestFixture<FakeReceiver> fixture(receiver);

    fixture.sendCommand({LEFT_RAD_S}, ARRIVAL_MS);

    TEST_ASSERT_EQUAL_UINT(0, receiver.received.size());
}

void test_ros_wheel_command_subscriber_drops_message_with_three_values()
{
    FakeReceiver receiver;
    SubscriberTestFixture<FakeReceiver> fixture(receiver);

    fixture.sendCommand({LEFT_RAD_S, RIGHT_RAD_S, RIGHT_RAD_S}, ARRIVAL_MS);

    TEST_ASSERT_EQUAL_UINT(0, receiver.received.size());
}

void test_ros_wheel_command_subscriber_drops_message_with_infinite_left_value()
{
    FakeReceiver receiver;
    SubscriberTestFixture<FakeReceiver> fixture(receiver);

    fixture.sendCommand({std::numeric_limits<float>::infinity(), RIGHT_RAD_S}, ARRIVAL_MS);

    TEST_ASSERT_EQUAL_UINT(0, receiver.received.size());
}

void test_ros_wheel_command_subscriber_drops_message_with_nan_right_value()
{
    FakeReceiver receiver;
    SubscriberTestFixture<FakeReceiver> fixture(receiver);

    fixture.sendCommand({LEFT_RAD_S, std::numeric_limits<float>::quiet_NaN()}, ARRIVAL_MS);

    TEST_ASSERT_EQUAL_UINT(0, receiver.received.size());
}

void test_ros_wheel_command_subscriber_preserves_timeout_for_setup_command()
{
    // The command arrives during setup(). The first step() comes TEST_TIMEOUT_MS later, so the
    // command is too old: the wheel control stops the motors and logs before it reads
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    SubscriberTestFixture<Control> fixture(control);
    fixture.sendCommand({LEFT_RAD_S, RIGHT_RAD_S}, ARRIVAL_MS);
    controller.clearCalls();

    control.step(ARRIVAL_MS + TEST_TIMEOUT_MS);

    TEST_ASSERT_EQUAL_STRING("eStop log read", controller.calls.c_str());
}

void test_ros_wheel_command_subscriber_does_not_restart_timeout_for_invalid_message()
{
    // A valid command, then a message with one value shortly before the valid command's
    // deadline. The dropped message doesn't restart the timeout, so the deadline stays
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    SubscriberTestFixture<Control> fixture(control);
    const uint32_t deadline_ms = ARRIVAL_MS + TEST_TIMEOUT_MS;
    const uint32_t invalid_arrival_ms = deadline_ms - 100;
    fixture.sendCommand({LEFT_RAD_S, RIGHT_RAD_S}, ARRIVAL_MS);
    fixture.sendCommand({LEFT_RAD_S}, invalid_arrival_ms);
    controller.clearCalls();

    control.step(deadline_ms);

    TEST_ASSERT_EQUAL_STRING("eStop log read", controller.calls.c_str());
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_ros_wheel_command_subscriber_passes_command_with_teensy_arrival_time);
    RUN_TEST(test_ros_wheel_command_subscriber_passes_every_command_in_same_millisecond);
    RUN_TEST(test_ros_wheel_command_subscriber_reaches_own_receiver_with_two_instances);
    RUN_TEST(test_ros_wheel_command_subscriber_drops_message_with_one_value);
    RUN_TEST(test_ros_wheel_command_subscriber_drops_message_with_three_values);
    RUN_TEST(test_ros_wheel_command_subscriber_drops_message_with_infinite_left_value);
    RUN_TEST(test_ros_wheel_command_subscriber_drops_message_with_nan_right_value);
    RUN_TEST(test_ros_wheel_command_subscriber_preserves_timeout_for_setup_command);
    RUN_TEST(test_ros_wheel_command_subscriber_does_not_restart_timeout_for_invalid_message);
    return UNITY_END();
}
