// Unit tests of RosWheelCommandSubscriber (lib/wheel_control/ros_wheel_command_subscriber.h) on
// the PC: rosserial's real parser with a fake serial port, and in the last tests a WheelControl
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

// rosserial numbers the subscribers from 100, in the order they subscribe
constexpr uint16_t WHEEL_COMMAND_TOPIC_ID = 100;

// The host's header stamp in the messages, far from the arrival times in the tests, so that a test
// fails if the subscriber takes the stamp as the arrival time
constexpr uint32_t HOST_STAMP_SEC = 1700000000;

// The wheel control's timing in the tests that pass the commands on to one, independent of the
// robot's settings: a command expires 1000 ms after it arrived
constexpr diffbot::WheelControlConfig TEST_CONFIG = {
    1000, // command_timeout_ms
    50,   // control_period_ms
};

typedef diffbot::WheelControl<FakeBaseController, FakeTimeoutLog> Control;

// The Teensy's millis(), one per test
struct TestClock
{
    uint32_t now_ms = 0;
    uint32_t millis() const { return now_ms; }
};

// A command as the subscriber passed it on
struct ReceivedCommand
{
    diffbot::WheelCommand command;
    uint32_t received_ms;
};

// Records the commands the subscriber passes on
struct FakeReceiver
{
    std::vector<ReceivedCommand> received;

    void receiveCommand(diffbot::WheelCommand command, uint32_t received_ms)
    {
        received.push_back({command, received_ms});
    }
};

// One test's node handle, clock and subscriber, which subscribes before initNode(), as setup()
// does before BaseController::setup()
template <class Receiver>
struct Subscription
{
    NodeHandle nh;
    TestClock clock;
    diffbot::RosWheelCommandSubscriber<Receiver, TestClock> subscriber;

    explicit Subscription(Receiver& receiver) : subscriber(receiver, clock)
    {
        subscriber.subscribe(nh);
        nh.initNode();
    }

    /// The host sends a wheel command message with these values, which arrives at now_ms, and
    /// loop() calls spinOnce()
    void receive(std::vector<float> values, uint32_t now_ms)
    {
        diffbot_msgs::WheelsCmdStamped message;
        message.header.stamp.sec = HOST_STAMP_SEC;
        message.wheels_cmd.angular_velocities.joint_length = static_cast<uint32_t>(values.size());
        message.wheels_cmd.angular_velocities.joint = values.data();
        // The header and up to three values need less than 64 bytes
        std::vector<uint8_t> data(64);
        data.resize(message.serialize(data.data()));

        clock.now_ms = now_ms;
        nh.getHardware()->receive(WHEEL_COMMAND_TOPIC_ID, data);
        nh.spinOnce();
    }
};

void setUp() {}
void tearDown() {}

void test_ros_wheel_command_subscriber_forwards_pair_with_local_receipt_time()
{
    FakeReceiver receiver;
    Subscription<FakeReceiver> subscription(receiver);

    subscription.receive({1.25f, -2.5f}, 123);

    TEST_ASSERT_EQUAL_UINT(1, receiver.received.size());
    TEST_ASSERT_EQUAL_FLOAT(1.25f, receiver.received[0].command.left_rad_s);
    TEST_ASSERT_EQUAL_FLOAT(-2.5f, receiver.received[0].command.right_rad_s);
    TEST_ASSERT_EQUAL_UINT32(123, receiver.received[0].received_ms);
}

void test_ros_wheel_command_subscriber_forwards_every_same_tick_arrival()
{
    FakeReceiver receiver;
    Subscription<FakeReceiver> subscription(receiver);

    subscription.receive({1.0f, 2.0f}, 123);
    subscription.receive({3.0f, 4.0f}, 123);

    TEST_ASSERT_EQUAL_UINT(2, receiver.received.size());
    TEST_ASSERT_EQUAL_FLOAT(3.0f, receiver.received[1].command.left_rad_s);
    TEST_ASSERT_EQUAL_UINT32(123, receiver.received[1].received_ms);
}

void test_ros_wheel_command_subscriber_reaches_own_receiver_with_two_instances()
{
    FakeReceiver first_receiver;
    FakeReceiver second_receiver;
    Subscription<FakeReceiver> first(first_receiver);
    Subscription<FakeReceiver> second(second_receiver);

    second.receive({3.0f, 4.0f}, 123);

    TEST_ASSERT_EQUAL_UINT(0, first_receiver.received.size());
    TEST_ASSERT_EQUAL_UINT(1, second_receiver.received.size());
}

void test_ros_wheel_command_subscriber_rejects_incomplete_pair()
{
    FakeReceiver receiver;
    Subscription<FakeReceiver> subscription(receiver);

    subscription.receive({1.0f}, 123);

    TEST_ASSERT_EQUAL_UINT(0, receiver.received.size());
}

void test_ros_wheel_command_subscriber_rejects_extra_values()
{
    FakeReceiver receiver;
    Subscription<FakeReceiver> subscription(receiver);

    subscription.receive({1.0f, 2.0f, 3.0f}, 123);

    TEST_ASSERT_EQUAL_UINT(0, receiver.received.size());
}

void test_ros_wheel_command_subscriber_rejects_nonfinite_left_value()
{
    FakeReceiver receiver;
    Subscription<FakeReceiver> subscription(receiver);

    subscription.receive({std::numeric_limits<float>::infinity(), 2.0f}, 123);

    TEST_ASSERT_EQUAL_UINT(0, receiver.received.size());
}

void test_ros_wheel_command_subscriber_rejects_nonfinite_right_value()
{
    FakeReceiver receiver;
    Subscription<FakeReceiver> subscription(receiver);

    subscription.receive({1.0f, std::numeric_limits<float>::quiet_NaN()}, 123);

    TEST_ASSERT_EQUAL_UINT(0, receiver.received.size());
}

void test_ros_wheel_command_subscriber_preserves_expiry_for_setup_receipt()
{
    // The command arrives during setup(), 1000 ms before the first loop()
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    Subscription<Control> subscription(control);
    subscription.receive({1.0f, -2.0f}, 100);
    controller.clearCalls();

    control.step(1100);

    TEST_ASSERT_EQUAL_STRING("eStop log read", controller.calls.c_str());
}

void test_ros_wheel_command_subscriber_does_not_refresh_expiry_for_invalid_pair()
{
    FakeBaseController controller;
    Control control(controller, TEST_CONFIG, FakeTimeoutLog{controller});
    Subscription<Control> subscription(control);
    subscription.receive({1.0f, -2.0f}, 0);
    subscription.receive({3.0f}, 900);
    controller.clearCalls();

    control.step(1000);

    TEST_ASSERT_EQUAL_STRING("eStop log read", controller.calls.c_str());
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_ros_wheel_command_subscriber_forwards_pair_with_local_receipt_time);
    RUN_TEST(test_ros_wheel_command_subscriber_forwards_every_same_tick_arrival);
    RUN_TEST(test_ros_wheel_command_subscriber_reaches_own_receiver_with_two_instances);
    RUN_TEST(test_ros_wheel_command_subscriber_rejects_incomplete_pair);
    RUN_TEST(test_ros_wheel_command_subscriber_rejects_extra_values);
    RUN_TEST(test_ros_wheel_command_subscriber_rejects_nonfinite_left_value);
    RUN_TEST(test_ros_wheel_command_subscriber_rejects_nonfinite_right_value);
    RUN_TEST(test_ros_wheel_command_subscriber_preserves_expiry_for_setup_receipt);
    RUN_TEST(test_ros_wheel_command_subscriber_does_not_refresh_expiry_for_invalid_pair);
    return UNITY_END();
}
