#include <stdint.h>

// Arduino's entry headers: millis(), micros(), TwoWire and Wire are declared in board-specific
// headers behind them (core_pins.h, WireKinetis.h or WireIMXRT.h)
#include <Arduino.h> // IWYU pragma: keep
#include <ros.h>
#include <Wire.h> // IWYU pragma: keep

#include "diffbot_base_config.h"
#include "base_controller.h"
#include "adafruit_feather_wing/adafruit_feather_wing.h"
#include "bno055.h"
#include "imu_publisher.h"
#include "ros_wheel_command_subscriber.h"
#include "synced_node_handle.h"
#include "wheel_control.h"
#include "wire_bus.h"


// The node handle also tells when its clock is synchronized with the host (synced_node_handle.h)
typedef diffbot::SyncedNodeHandleT<ros::NodeHandle> NodeHandle;
NodeHandle nh;

using namespace diffbot;

AdafruitMotorController motor_controller_right = AdafruitMotorController(MOTOR_RIGHT);
AdafruitMotorController motor_controller_left = AdafruitMotorController(MOTOR_LEFT);

BaseController<AdafruitMotorController, Adafruit_MotorShield> base_controller(nh, &motor_controller_left, &motor_controller_right);

// The wheel control and the IMU, private to this file
namespace
{
    // The Arduino clock for the IMU publisher and the wheel commands' arrival times, which the
    // unit tests replace with a fake one
    struct ArduinoClock
    {
        static uint32_t millis() { return ::millis(); }
        static uint32_t micros() { return ::micros(); }
    };

    // Logs the command timeout, after WheelControl has stopped the motors
    struct RosTimeoutLogger
    {
        NodeHandle& nh;
        void operator()() { nh.logwarn("Wheel command timeout: motors stopped"); }
    };

    // The wheel control's timing. The control period is in whole milliseconds: 50 ms at 20 Hz
    constexpr WheelControlConfig WHEEL_CONTROL_CONFIG = {
        COMMAND_TIMEOUT_MS,                                  // command_timeout_ms
        static_cast<uint32_t>(1000.0 / UPDATE_RATE_CONTROL), // control_period_ms
    };
    static_assert(WHEEL_CONTROL_CONFIG.valid(),
                  "WHEEL_CONTROL_CONFIG: check COMMAND_TIMEOUT_MS and UPDATE_RATE_CONTROL");

    // Drives the wheels only while the wheel commands from the robot's computer are new: the
    // subscriber passes each valid command on to the wheel control, loop() calls its step
    ArduinoClock wheel_clock;
    WheelControl<decltype(base_controller), RosTimeoutLogger> wheel_control(
        base_controller, WHEEL_CONTROL_CONFIG, RosTimeoutLogger{nh});
    RosWheelCommandSubscriber<decltype(wheel_control), ArduinoClock> wheel_commands(
        wheel_control, wheel_clock);

    // The IMU, on the same I2C bus as the motor driver. It publishes imu/data_raw, which rosserial
    // puts in its node's namespace (/diffbot/imu/data_raw). The robot drives without an IMU too.
    // IMU_PUBLISHER_CONFIG in diffbot_base_config.h turns it off for a robot without one.
    WireBusT<TwoWire> wire_bus(Wire);
    Bno055 imu(wire_bus, IMU_BNO055_ADDRESS);
    ImuPublisher<NodeHandle, ArduinoClock> imu_publisher(nh, imu, IMU_PUBLISHER_CONFIG);
}


void setup()
{
    // Subscribe and advertise before base_controller.setup() connects: rosserial announces the
    // topics then. Wheel commands that arrive during the setup reach the wheel control already
    wheel_commands.subscribe(nh);
    imu_publisher.advertise();
    Wire.begin();
    base_controller.setup();
    base_controller.init();

    nh.loginfo("Initialize DiffBot Motor Controllers");
    motor_controller_left.begin();
    motor_controller_right.begin();
    nh.loginfo("Setup finished");
}


void loop()
{
    // Use the Teensy's elapsed time for scheduling (see BaseController::LastUpdateTime).
    const uint32_t now_ms = millis();

    // Stops the motors when the wheel commands stop arriving, and runs the control step
    wheel_control.step(now_ms);

    imu_publisher.update();

    // This block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(base_controller.debug())
    {
        if (now_ms - base_controller.lastUpdateTime().debug >= base_controller.publishRate().period().debug_ * 1000.0)
        {
            base_controller.printDebug();
            base_controller.lastUpdateTime().debug = now_ms;
        }
    }
    // Call all the callbacks waiting to be called. Last, after the wheel control's step: a wheel
    // command that arrived after now_ms was read would look about 49.7 days old to the step
    nh.spinOnce();
}