// The parts of BaseController that WheelControl uses (lib/wheel_control/wheel_control.h), and its
// timeout log, for the native unit tests. Each test makes its own, so that it sees only its calls.
// They check how WheelControl calls the controller, not what BaseController::eStop() itself does.
#ifndef DIFFBOT_FAKE_BASE_CONTROLLER_H
#define DIFFBOT_FAKE_BASE_CONTROLLER_H

#include <string>
#include <vector>

#include "wheel_command.h"

// The motor output that write() sets: the PID's output while the wheels are driven, not zero
constexpr int DRIVING_MOTOR_OUTPUT = 100;

/// Records its calls in order, for example "setWheelTargets read write". motor_output is what the
/// real controller sets the motors to: the PID's output after write(), zero after eStop()
struct FakeBaseController
{
    std::string calls;
    diffbot::WheelCommand targets{0.0f, 0.0f};
    int motor_output = 0;
    /// motor_output at each timeout log message (FakeTimeoutLog)
    std::vector<int> motor_outputs_at_log;

    void setWheelTargets(diffbot::WheelCommand command)
    {
        targets = command;
        record("setWheelTargets");
    }

    void read() { record("read"); }

    void write()
    {
        motor_output = DRIVING_MOTOR_OUTPUT;
        record("write");
    }

    void eStop()
    {
        targets = {0.0f, 0.0f};
        motor_output = 0;
        record("eStop");
    }

    void record(const char* call)
    {
        if (!calls.empty())
            calls += ' ';
        calls += call;
    }

    /// Forgets what was recorded so far, so that a test checks only what its action does
    void clearCalls()
    {
        calls.clear();
        motor_outputs_at_log.clear();
    }
};

/// WheelControl's timeout log: records "log" among the controller's calls, so that a test sees
/// the order, and the motor output at that moment
struct FakeTimeoutLog
{
    FakeBaseController& controller;

    void operator()()
    {
        controller.record("log");
        controller.motor_outputs_at_log.push_back(controller.motor_output);
    }
};

#endif // DIFFBOT_FAKE_BASE_CONTROLLER_H
