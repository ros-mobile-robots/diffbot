#ifndef DIFFBOT_ROS_WHEEL_COMMAND_SUBSCRIBER_H
#define DIFFBOT_ROS_WHEEL_COMMAND_SUBSCRIBER_H

#include <cmath>

#include <diffbot_msgs/WheelsCmdStamped.h>
#include <ros/subscriber.h>

#include "wheel_command.h"

namespace diffbot
{
    /**
     * @brief Subscribes to wheel_cmd_velocities and passes each valid wheel command to the wheel
     * control.
     *
     * On the robot's computer, DiffBotHWInterface::write() (diffbot_base) publishes the target
     * speeds of the left and the right wheel, in rad/s. rosserial sends each message over USB to
     * the Teensy. There, nh.spinOnce() reads the message and calls commandCallback(), which:
     * 1. Drops a message that doesn't hold exactly two finite speeds. A dropped message changes
     *    neither the targets nor the command timeout.
     * 2. Reads the Teensy's clock (Clock::millis()) when the callback runs, as the arrival time.
     *    The timeout starts at that processing time, not at the message's ROS header stamp,
     *    which jumps when the computer's clock is corrected.
     * 3. Calls Receiver::receiveCommand() with the command and its arrival time in ms.
     *
     * In ports and adapters terms, RosWheelCommandSubscriber is a primary adapter. The callback
     * converts a rosserial message into a WheelCommand and calls WheelControl's primary port
     * receiveCommand(). Receiver and Clock are template type parameters. The unit tests supply
     * a fake receiver and a fake clock:
     * - Receiver: has receiveCommand(WheelCommand, uint32_t). WheelControl on the robot.
     * - Clock: has millis(), the milliseconds since the Teensy started. ArduinoClock on the robot.
     *
     * @tparam Receiver See above
     * @tparam Clock See above
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

        /// Registers the subscriber with the node handle. Call it before BaseController::setup()
        /// waits for the connection: when spinOnce() receives the computer's topic-list request,
        /// rosserial answers with the topics registered so far, and the computer forwards wheel
        /// commands only for a subscriber in that answer
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
