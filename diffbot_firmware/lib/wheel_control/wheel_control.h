#ifndef DIFFBOT_WHEEL_CONTROL_H
#define DIFFBOT_WHEEL_CONTROL_H

#include <stdint.h>

#include "command_timeout.h"
#include "wheel_command.h"

namespace diffbot
{
    /// The timing of \ref WheelControl, fixed for its lifetime
    struct WheelControlConfig
    {
        uint32_t command_timeout_ms;  ///< Stop the motors after this long without a new command
        uint32_t control_period_ms;   ///< Read the encoders and drive the motors this often

        /// True if both are longer than zero
        constexpr bool valid() const
        {
            return command_timeout_ms > 0 && control_period_ms > 0;
        }
    };

    /**
     * @brief Controls the wheel updates and stops the motors when the wheel commands stop.
     *
     * The primary ports are the functions that other code calls:
     * - receiveCommand() takes two valid target speeds and the command's receipt time in ms.
     *   The function stores the targets and restarts the command timeout without writing motor
     *   outputs. Commands can arrive before the first step().
     * - step() runs on every pass of loop(), with the current millis() value from loop(). Each
     *   call does the following work in order:
     *   1. Checks the last command's age using now_ms. On the first check that finds the command
     *      expired, calls Controller::eStop() and then LogTimeout. The stop comes first because
     *      the log message goes over USB, and that write can wait for transmit space.
     *   2. Returns if the control period has not elapsed since the previous control step.
     *   3. Calls Controller::read(), also while driving is disabled.
     *   4. Calls Controller::write() only if the timeout check permits driving.
     *   5. Records now_ms as the time of this control step.
     *
     * Before the first command, step() reads on schedule without writing motor outputs or
     * logging a timeout. After a timeout, a new valid command permits driving again, if that
     * command has not expired when a scheduled control step checks the timeout.
     *
     * The secondary ports are the operations this class calls on its template arguments:
     * - Controller:
     *   - setWheelTargets(WheelCommand): stores both targets without writing motor outputs.
     *   - read(): reads the encoders and publishes wheel positions and velocities.
     *   - write(): computes both PID outputs and sends those outputs to the motor controllers.
     *   - eStop(): clears both targets, resets both PIDs and sets both motor outputs to zero.
     * - LogTimeout: a callable with no arguments that logs the timeout.
     *
     * The unit tests supply fake implementations of the secondary ports. The firmware's README
     * (diffbot_firmware/README.md) explains the pattern and names the implementations used on
     * the robot.
     *
     * @tparam Controller Implements the controller operations listed above
     * @tparam LogTimeout Implements the timeout log operation listed above
     */
    template <class Controller, class LogTimeout>
    class WheelControl
    {
    public:
        /// config.valid() must be true. The referenced controller must outlive this object.
        /// log_timeout is copied. Objects referenced by that copy must also outlive this object
        WheelControl(Controller& controller, WheelControlConfig config, LogTimeout log_timeout)
            : controller_(controller), timeout_(config.command_timeout_ms),
              control_period_ms_(config.control_period_ms), log_timeout_(log_timeout)
        {
        }

        // Not copyable: the commands have to reach this object, not a copy
        WheelControl(const WheelControl&) = delete;
        WheelControl& operator=(const WheelControl&) = delete;

        /// Stores a valid command and restarts the timeout without writing motor outputs.
        /// received_ms is the Teensy's millis() when the caller received the command.
        /// step() checks that command's age before allowing a scheduled motor write
        void receiveCommand(WheelCommand command, uint32_t received_ms)
        {
            controller_.setWheelTargets(command);
            timeout_.commandArrived(received_ms);
        }

        /// Call once per loop pass with a current millis() reading taken after command callbacks
        void step(uint32_t now_ms)
        {
            stopWhenCommandExpires(now_ms);
            if (now_ms - last_control_ms_ < control_period_ms_)
                return;
            controller_.read();
            if (!timeout_.timedOut())
                controller_.write();
            last_control_ms_ = now_ms;
        }

    private:
        /// Checks the timeout on every call. On the first call that finds the last command
        /// expired, stops the motors and then logs, once
        void stopWhenCommandExpires(uint32_t now_ms)
        {
            const bool was_timed_out = timeout_.timedOut();
            timeout_.update(now_ms);
            if (!was_timed_out && timeout_.timedOut())
            {
                controller_.eStop();
                log_timeout_();
            }
        }

        Controller& controller_;
        CommandTimeout timeout_;
        const uint32_t control_period_ms_;
        LogTimeout log_timeout_;
        /// The first control step is due one period after millis() starts at zero
        uint32_t last_control_ms_ = 0;
    };
}

#endif // DIFFBOT_WHEEL_CONTROL_H
