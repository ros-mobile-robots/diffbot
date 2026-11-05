#ifndef DIFFBOT_COMMAND_TIMEOUT_H
#define DIFFBOT_COMMAND_TIMEOUT_H

#include <stdint.h>

namespace diffbot {

    /**
     * @brief Tracks whether wheel commands permit motor drive.
     *
     * commandArrived(received_ms) records a valid command's receipt time and clears the timeout.
     * update(now_ms) sets the timeout when the recorded command is at least timeout_ms old.
     * timedOut() reports that stored state. The caller supplies Teensy millis() readings rather
     * than ROS timestamps, so changes in the SBC's ROS time do not affect the timeout.
     *
     * The initial state is timed out, which prevents driving before the first command.
     * After a command expires, the state stays timed out until commandArrived() is called again.
     * This latch prevents an old command from becoming usable when millis() repeats a previous
     * value after a full cycle of about 49.7 days.
     *
     * This class does not access hardware. WheelControl detects the transition into a timeout,
     * clears the motor outputs and logs once. Unit tests exercise this class on the PC.
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

        /// Sets the timeout when the last command is at least timeout_ms old.
        /// Unsigned subtraction handles a counter wrap if less than one full cycle has elapsed
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
