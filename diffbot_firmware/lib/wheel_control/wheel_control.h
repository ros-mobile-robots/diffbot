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
     * @brief Drives the wheels only while the wheel commands are new.
     *
     * \ref receiveCommand stores a command's targets and tells the private \ref CommandTimeout
     * that a command arrived. \ref step, called in every loop(), checks the timeout first: when
     * the last command has become too old, it stops the motors (Controller::eStop) and only then
     * logs, once per timeout, because the log message can wait for USB. Then, every control
     * period, it reads and publishes the wheel states, also while timed out, and drives the
     * motors (Controller::write) only while the last command is new.
     *
     * The subscriber (RosWheelCommandSubscriber) calls receiveCommand from rosserial's spinOnce(),
     * including while setup waits for parameters. In loop(), spinOnce runs last and the next
     * pass calls step with a new time. If the latest accepted command is already too old at the
     * first step, step stops the motors and logs before any PID write. Before the first command,
     * step neither drives nor stops nor logs.
     *
     * Ports: the functions through which the rest of the firmware uses this class, and the ones
     * this class needs from it. Adapters connect them to rosserial and to the hardware.
     * - Offered (inbound): receiveCommand(), called by the subscriber (RosWheelCommandSubscriber),
     *   and step(), called by loop().
     * - Needed (outbound), the Controller, BaseController on the robot:
     *   - setWheelTargets(WheelCommand): stores both targets, without driving the motors.
     *   - read(): reads the encoders and publishes the wheel states.
     *   - write(): runs both PIDs towards the targets and drives the motors.
     *   - eStop(): sets both targets to zero, resets both PIDs and sets both motors to zero,
     *     which the FeatherWing turns into releasing them.
     * - Needed (outbound), the LogTimeout: a callable without arguments that logs the timeout.
     * Both are template parameters, so that the unit tests replace them with fakes
     * (test/test_wheel_control) and the robot's build calls them directly.
     *
     * @tparam Controller See the ports above
     * @tparam LogTimeout See the ports above
     */
    template <class Controller, class LogTimeout>
    class WheelControl
    {
    public:
        /// config must be valid(). The controller and what the log uses must outlive this object
        WheelControl(Controller& controller, WheelControlConfig config, LogTimeout log_timeout)
            : controller_(controller), timeout_(config.command_timeout_ms),
              control_period_ms_(config.control_period_ms), log_timeout_(log_timeout)
        {
        }

        // Not copyable: the subscriber tells this object, not a copy, about new commands
        WheelControl(const WheelControl&) = delete;
        WheelControl& operator=(const WheelControl&) = delete;

        /// Stores a complete, checked command that arrived at received_ms. It doesn't drive the
        /// motors: the next control step does
        void receiveCommand(WheelCommand command, uint32_t received_ms)
        {
            controller_.setWheelTargets(command);
            timeout_.commandArrived(received_ms);
        }

        /// Call in every loop(), with millis()
        void step(uint32_t now_ms)
        {
            enforceTimeout(now_ms);
            if (now_ms - last_control_ms_ < control_period_ms_)
                return;
            controller_.read();
            if (!timeout_.timedOut())
                controller_.write();
            last_control_ms_ = now_ms;
        }

    private:
        /// Stops the motors, then logs, on the step when the last command becomes too old
        void enforceTimeout(uint32_t now_ms)
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
