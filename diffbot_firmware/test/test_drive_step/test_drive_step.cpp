// Unit tests of driveStep (lib/drive_step/drive_step.h), loop()'s command timeout and control
// step, with a fake base controller. Run them with:
//   pio test -e native
#include <unity.h>

#include <stdint.h>

#include <string>
#include <vector>

#include "drive_step.h"

void setUp() {}
void tearDown() {}

static std::vector<std::string> calls;  // what the fake controller and the log saw, in order

// The parts of BaseController that driveStep uses. eStop() and write() record the motor output
// the real ones set: zero after eStop(), the PID's output after write().
struct FakeBaseController
{
    struct Times
    {
        uint32_t command_received = 0;
        uint32_t control = 0;
    } times;
    uint32_t command_count = 0;
    int motor_output = 0;
    int pid_output = 255;  // a wound-up PID

    Times& lastUpdateTime() { return times; }
    uint32_t commandCount() const { return command_count; }
    void read() { calls.push_back("read"); }
    void write() { motor_output = pid_output; calls.push_back("write"); }
    void eStop() { motor_output = 0; pid_output = 0; calls.push_back("eStop"); }
    void command(uint32_t now_ms)
    {
        times.command_received = now_ms;
        ++command_count;
        pid_output = 100;
    }
};

static const uint32_t TIMEOUT_MS = 1000;
static const uint32_t PERIOD_MS = 50;

static void step(FakeBaseController& controller, diffbot::CommandTimeout& timeout, uint32_t now_ms)
{
    diffbot::driveStep(controller, timeout, now_ms, PERIOD_MS, [&controller]() {
        // The motors are already stopped when the log message is sent
        TEST_ASSERT_EQUAL_INT(0, controller.motor_output);
        calls.push_back("log");
    });
}

void test_drive_step_doesnt_drive_before_the_first_command()
{
    calls.clear();
    FakeBaseController controller;
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    step(controller, timeout, 50);
    step(controller, timeout, 100);
    TEST_ASSERT_EQUAL_INT(2, (int)calls.size());
    TEST_ASSERT_EQUAL_STRING("read", calls[0].c_str());
    TEST_ASSERT_EQUAL_STRING("read", calls[1].c_str());
    TEST_ASSERT_EQUAL_INT(0, controller.motor_output);
}

void test_drive_step_stops_the_motors_before_logging_and_keeps_reading_on_timeout()
{
    calls.clear();
    FakeBaseController controller;
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    controller.command(100);
    step(controller, timeout, 150);  // drives
    TEST_ASSERT_EQUAL_INT(100, controller.motor_output);
    controller.pid_output = 255;     // the integral winds up while a wheel is held back

    calls.clear();
    step(controller, timeout, 1100);  // the command is 1000 ms old
    TEST_ASSERT_EQUAL_INT(3, (int)calls.size());
    TEST_ASSERT_EQUAL_STRING("eStop", calls[0].c_str());
    TEST_ASSERT_EQUAL_STRING("log", calls[1].c_str());
    TEST_ASSERT_EQUAL_STRING("read", calls[2].c_str());
    TEST_ASSERT_EQUAL_INT(0, controller.motor_output);

    // Later control steps read and publish, but don't drive, and don't log again
    calls.clear();
    step(controller, timeout, 1150);
    step(controller, timeout, 1200);
    TEST_ASSERT_EQUAL_INT(2, (int)calls.size());
    TEST_ASSERT_EQUAL_STRING("read", calls[0].c_str());
    TEST_ASSERT_EQUAL_STRING("read", calls[1].c_str());
    TEST_ASSERT_EQUAL_INT(0, controller.motor_output);
}

void test_drive_step_drives_again_after_the_next_command()
{
    calls.clear();
    FakeBaseController controller;
    diffbot::CommandTimeout timeout(TIMEOUT_MS);
    controller.command(100);
    step(controller, timeout, 150);
    step(controller, timeout, 1100);  // timed out
    controller.command(1120);
    calls.clear();
    step(controller, timeout, 1150);
    TEST_ASSERT_EQUAL_INT(2, (int)calls.size());
    TEST_ASSERT_EQUAL_STRING("read", calls[0].c_str());
    TEST_ASSERT_EQUAL_STRING("write", calls[1].c_str());
    TEST_ASSERT_EQUAL_INT(100, controller.motor_output);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_drive_step_doesnt_drive_before_the_first_command);
    RUN_TEST(test_drive_step_stops_the_motors_before_logging_and_keeps_reading_on_timeout);
    RUN_TEST(test_drive_step_drives_again_after_the_next_command);
    return UNITY_END();
}
