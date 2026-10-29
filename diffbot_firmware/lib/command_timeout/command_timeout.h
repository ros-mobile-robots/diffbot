#ifndef DIFFBOT_COMMAND_TIMEOUT_H
#define DIFFBOT_COMMAND_TIMEOUT_H

#include <stdint.h>

namespace diffbot {

    /** \brief Decides when the wheel commands from the robot's computer are too old.
     *
     * loop() calls \ref update on every pass, before the control step, with the Teensy's elapsed
     * milliseconds (millis()), the time the last wheel command arrived, and how many commands
     * have arrived so far. When no command has arrived for the timeout, update() returns true
     * once, so that the motors are stopped and the timeout is logged once, not on every pass.
     * While \ref timedOut is true, the control step doesn't drive the motors.
     *
     * Only a new command ends the timeout, recognized by the changed command count, not by its
     * time: after about 49.7 days, millis() returns to zero, and an old time would look new again.
     * It starts timed out, so the motors aren't driven before the first command arrives.
     *
     * Plain C++ without Arduino or rosserial, so the unit tests run it on the PC.
     */
    class CommandTimeout
    {
    public:
        explicit CommandTimeout(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}

        /// Returns true once, on the pass when the last command becomes too old. The unsigned
        /// difference stays right when millis() returns to zero.
        bool update(uint32_t now_ms, uint32_t last_command_ms, uint32_t command_count)
        {
            if (timed_out_)
            {
                if (command_count == count_at_timeout_)
                {
                    return false;
                }
                timed_out_ = false;  // a new command arrived
            }
            if (now_ms - last_command_ms >= timeout_ms_)
            {
                timed_out_ = true;
                count_at_timeout_ = command_count;
                return true;
            }
            return false;
        }

        /// True from the timeout until the next command arrives, and before the first command
        bool timedOut() const { return timed_out_; }

    private:
        uint32_t timeout_ms_;
        bool timed_out_ = true;
        uint32_t count_at_timeout_ = 0;
    };
}

#endif // DIFFBOT_COMMAND_TIMEOUT_H
