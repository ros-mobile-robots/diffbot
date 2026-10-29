#ifndef DIFFBOT_COMMAND_TIMEOUT_H
#define DIFFBOT_COMMAND_TIMEOUT_H

#include <stdint.h>

namespace diffbot {

    /** \brief Decides when the wheel commands from the robot's computer are too old.
     *
     * loop() calls \ref update on every pass, before the control step, with the Teensy's elapsed
     * milliseconds (millis()) and the time the last wheel command arrived. When no command has
     * arrived for the timeout, update() returns true once, so that the motors are stopped and the
     * timeout is logged once, not on every pass. While \ref timedOut is true, the control step
     * doesn't drive the motors. The next command ends the timeout.
     *
     * Plain C++ without Arduino or rosserial, so the unit tests run it on the PC.
     */
    class CommandTimeout
    {
    public:
        explicit CommandTimeout(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}

        /// Returns true once, on the pass when the last command becomes too old. The unsigned
        /// difference stays right when millis() returns to zero after about 49.7 days.
        bool update(uint32_t now_ms, uint32_t last_command_ms)
        {
            const bool too_old = now_ms - last_command_ms >= timeout_ms_;
            if (too_old && !timed_out_)
            {
                timed_out_ = true;
                return true;
            }
            if (!too_old)
            {
                timed_out_ = false;
            }
            return false;
        }

        /// True from the timeout until the next command arrives
        bool timedOut() const { return timed_out_; }

    private:
        uint32_t timeout_ms_;
        bool timed_out_ = false;
    };
}

#endif // DIFFBOT_COMMAND_TIMEOUT_H
