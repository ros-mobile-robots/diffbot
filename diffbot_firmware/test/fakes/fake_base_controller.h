// Fake implementations of WheelControl's controller and timeout-log ports for the native tests.
// Each test constructs fresh fakes and inspects their recorded calls and simulated motor output.
// These tests check WheelControl's use of the ports. BaseController::eStop() is not executed.
#ifndef DIFFBOT_FAKE_BASE_CONTROLLER_H
#define DIFFBOT_FAKE_BASE_CONTROLLER_H

#include <string>
#include <vector>

#include "wheel_command.h"

// A nonzero marker that the fake's write() sets as the motor output. It makes a missing stop
// visible: after eStop() the output is zero again
constexpr int DRIVING_MOTOR_OUTPUT = 100;

/// Records its calls in order, for example "setWheelTargets read write". motor_output is a
/// simulated motor output: DRIVING_MOTOR_OUTPUT after write(), zero after eStop()
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

    /// Clears the call trace and the logged output samples, so that a test checks only what its
    /// action does. Leaves the targets and the motor output unchanged
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
