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
     * pass calls step with a new time. If the last valid command is already too old at the first
     * step, step stops the motors and logs before either PID computes a motor output. Before the
     * first command, step neither drives nor stops nor logs.
     *
     * Ports and adapters (Alistair Cockburn's hexagonal architecture): a port describes an
     * interaction through one or more functions and their required behavior. An adapter connects
     * those functions to external code or hardware. Here, WheelControl is the core. Cockburn
     * calls the two kinds of ports primary and secondary. Other texts say driving and driven, or
     * inbound and outbound.
     *
     * Primary ports: functions that other code calls to use WheelControl.
     * - receiveCommand(): RosWheelCommandSubscriber, the primary adapter for rosserial, calls
     *   this function with each valid command and the command's receipt time.
     * - step(): loop() calls this function on every pass, with the current time in milliseconds.
     *
     * Secondary ports: functions that WheelControl calls, supplied by other code.
     * - Controller, implemented by BaseController on the robot:
     *   - setWheelTargets(WheelCommand): stores both targets without changing the motor outputs.
     *   - read(): reads the encoders and publishes the wheel positions and velocities.
     *   - write(): computes both PID outputs from the targets and measured velocities, then
     *     sends the outputs to the motor controllers.
     *   - eStop(): sets both targets to zero, resets both PIDs and sets both motor outputs to
     *     zero. On the FeatherWing, zero output releases each motor, so the wheels can coast.
     * - LogTimeout: a callable with no arguments, called after eStop() returns. On the robot,
     *   RosTimeoutLogger in main.cpp logs the timeout through nh.logwarn().
     *
     * Controller and LogTimeout are template type parameters. The robot uses BaseController
     * and RosTimeoutLogger, so calls from WheelControl to these functions are nonvirtual.
     * The unit tests supply FakeBaseController and FakeTimeoutLog instead, through the same
     * ports. See test/test_wheel_control.
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

        /// Stores a valid command and restarts its timeout. received_ms is the Teensy's millis()
        /// when the subscriber processed the command. A due control step drives the motors only
        /// while the command is still fresh
        void receiveCommand(WheelCommand command, uint32_t received_ms)
        {
            controller_.setWheelTargets(command);
            timeout_.commandArrived(received_ms);
        }

        /// Call in every loop(), with millis()
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
