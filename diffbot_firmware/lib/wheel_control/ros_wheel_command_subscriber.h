#ifndef DIFFBOT_ROS_WHEEL_COMMAND_SUBSCRIBER_H
#define DIFFBOT_ROS_WHEEL_COMMAND_SUBSCRIBER_H

#include <cmath>

#include <diffbot_msgs/WheelsCmdStamped.h>
#include <ros/subscriber.h>

#include "wheel_command.h"

namespace diffbot
{
    /**
     * @brief Receives the wheel commands on wheel_cmd_velocities and passes each valid one on to
     * its receiver, with the time it arrived.
     *
     * diffbot_base's DiffBotHWInterface::write() publishes the commands, left and right wheel in
     * rad/s. A message with exactly two finite values is valid. Any other message is dropped: it
     * changes neither the targets nor the command timeout.
     *
     * The arrival time is the Teensy's clock (Clock::millis), not the message's header stamp: the
     * command timeout measures how long ago a command arrived on the Teensy, and ROS time jumps
     * when the host's clock is corrected.
     *
     * rosserial calls the callback in the node handle's spinOnce().
     *
     * An adapter: it connects rosserial to the wheel control's inbound port. Its own ports, both
     * template parameters so that the unit tests replace them:
     * - The Receiver, WheelControl on the robot: receiveCommand(WheelCommand, uint32_t), with a
     *   valid command and the time it arrived in ms.
     * - The Clock: millis(), the Teensy's milliseconds since the start, as the Arduino function.
     *
     * @tparam Receiver See the ports above
     * @tparam Clock See the ports above
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

        /// Call before the node handle connects, so that rosserial announces the topic then, and
        /// commands arriving during the setup already reach the receiver
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
