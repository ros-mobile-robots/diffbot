#ifndef DIFFBOT_ROS_WHEEL_COMMAND_SUBSCRIBER_H
#define DIFFBOT_ROS_WHEEL_COMMAND_SUBSCRIBER_H

#include <cmath>

#include <diffbot_msgs/WheelsCmdStamped.h>
#include <ros/subscriber.h>

#include "wheel_command.h"

namespace diffbot
{
    /**
     * @brief Receives wheel_cmd_velocities and passes valid commands to the receiver.
     *
     * How a command reaches commandCallback():
     * 1. DiffBotHWInterface::write() publishes the left and right target speeds in rad/s on the
     *    robot's single-board computer (SBC).
     * 2. serial_node.py, which is part of rosserial_python, forwards the message over USB serial.
     *    The received bytes wait in the Teensy's USB serial receive buffers.
     * 3. The node handle's spinOnce() reads and parses those bytes. The message's topic ID
     *    selects a subscriber from the node handle's list of registered subscribers.
     * 4. For wheel commands, that subscriber is subscriber_. The constructor below binds
     *    subscriber_ to this object's commandCallback(). subscriber_ decodes the message and
     *    calls that function.
     *
     * In this firmware, commandCallback() runs inside spinOnce(), never in a USB interrupt.
     * loop() calls spinOnce() last. BaseController::setup() also calls spinOnce() while waiting
     * for the connection. During BaseController::init(), rosserial calls spinOnce() while
     * waiting for parameter replies. Wheel commands can reach the callback during those waits.
     *
     * commandCallback() does three things:
     * 1. Drops a message unless the message holds exactly two finite speeds. A dropped message
     *    changes neither the targets nor the command timeout. Two zero speeds form a valid
     *    command.
     * 2. Reads Clock::millis() after these checks. WheelControl measures the command's age from
     *    this processing time, which can be later than USB reception. The ROS header stamp is
     *    unused, so changes in the SBC's ROS time don't affect the age.
     * 3. Calls Receiver::receiveCommand() with both speeds and the receipt time in milliseconds.
     *
     * This class is a primary adapter: the callback calls WheelControl's primary port
     * receiveCommand(). The firmware's README (diffbot_firmware/README.md) explains ports and
     * adapters. The unit tests supply fakes for both template parameters.
     *
     * @tparam Receiver Provides receiveCommand(WheelCommand, uint32_t). WheelControl on the robot
     * @tparam Clock Provides millis(), the wrapping millisecond counter. ArduinoClock on the robot
     */
    template <class Receiver, class Clock>
    class RosWheelCommandSubscriber
    {
    public:
        /// The receiver and the clock must outlive this object
        RosWheelCommandSubscriber(Receiver& receiver, Clock& clock)
            : receiver_(receiver), clock_(clock),
              subscriber_("wheel_cmd_velocities", &RosWheelCommandSubscriber::commandCallback,
                          this)
        {
        }

        // Not copyable: rosserial keeps the address of subscriber_, which calls this object
        RosWheelCommandSubscriber(const RosWheelCommandSubscriber&) = delete;
        RosWheelCommandSubscriber& operator=(const RosWheelCommandSubscriber&) = delete;

        /// Registers subscriber_ with the node handle before the initial topic negotiation.
        /// Call before BaseController::setup(). During setup, serial_node.py asks for the
        /// Teensy's topics. The node handle sends one TopicInfo message for each registered
        /// publisher and subscriber. For subscriber_, serial_node.py creates a ROS subscriber on
        /// the SBC that forwards wheel_cmd_velocities messages to the Teensy
        template <class NodeHandle>
        void subscribe(NodeHandle& nh)
        {
            nh.subscribe(subscriber_);
        }

    private:
        void commandCallback(const diffbot_msgs::WheelsCmdStamped& message)
        {
            const auto& velocities = message.wheels_cmd.angular_velocities;
            if (velocities.joint_length != 2 || velocities.joint == nullptr)
                return;
            const WheelCommand command{velocities.joint[0], velocities.joint[1]};
            if (!std::isfinite(command.left_rad_s) || !std::isfinite(command.right_rad_s))
                return;
            receiver_.receiveCommand(command, clock_.millis());
        }

        Receiver& receiver_;
        Clock& clock_;
        ros::Subscriber<diffbot_msgs::WheelsCmdStamped, RosWheelCommandSubscriber> subscriber_;
    };
}

#endif // DIFFBOT_ROS_WHEEL_COMMAND_SUBSCRIBER_H
