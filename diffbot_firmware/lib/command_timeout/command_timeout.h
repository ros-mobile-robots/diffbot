#ifndef DIFFBOT_COMMAND_TIMEOUT_H
#define DIFFBOT_COMMAND_TIMEOUT_H

#include <stdint.h>

namespace diffbot {

    /** \brief Decides when the last wheel command is too old to drive the motors.
     *
     * \ref commandArrived tells it when a command arrived, \ref update compares that time with the
     * current one, and \ref timedOut answers whether the command is too old. The times are the
     * Teensy's elapsed milliseconds (millis()), not ROS time, which jumps when the host's clock is
     * corrected.
     *
     * Only a new command ends the timeout, not the time: after about 49.7 days, millis() returns
     * to zero, and an old command's time would look new again. It starts timed out, so the motors
     * aren't driven before the first command arrives.
     *
     * It only decides: WheelControl stops the motors and logs, once, when timedOut() turns true
     * in an update().
     *
     * Plain C++ without Arduino or rosserial, so the unit tests run it on the PC.
     */
    class CommandTimeout
    {
    public:
        explicit CommandTimeout(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}

        /// A command arrived at received_ms: not timed out until the next update() at least
        /// timeout_ms later
        void commandArrived(uint32_t received_ms)
        {
            last_command_ms_ = received_ms;
            timed_out_ = false;
        }

        /// Times out when the last command is timeout_ms old. The unsigned difference stays right
        /// when millis() returns to zero
        void update(uint32_t now_ms)
        {
            if (!timed_out_ && now_ms - last_command_ms_ >= timeout_ms_)
                timed_out_ = true;
        }

        /// True before the first command, and from a timeout until the next command arrives
        bool timedOut() const { return timed_out_; }

    private:
        const uint32_t timeout_ms_;
        uint32_t last_command_ms_ = 0;
        bool timed_out_ = true;
    };
}

#endif // DIFFBOT_COMMAND_TIMEOUT_H
