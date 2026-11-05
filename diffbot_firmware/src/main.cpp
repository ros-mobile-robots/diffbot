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

// The wheel control, the IMU and the debug output, private to this file
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

    // WheelControl's timing, from the settings in diffbot_base_config.h: the command timeout, and
    // the control period in whole milliseconds (50 ms at UPDATE_RATE_CONTROL = 20 Hz).
    // The static_assert stops the build if a setting makes either value zero. The check is here,
    // where the settings are constant expressions: WheelControl takes its config as an ordinary
    // constructor argument, which static_assert can't check.
    constexpr WheelControlConfig WHEEL_CONTROL_CONFIG = {
        COMMAND_TIMEOUT_MS,                                  // command_timeout_ms
        static_cast<uint32_t>(1000.0 / UPDATE_RATE_CONTROL), // control_period_ms
    };
    static_assert(WHEEL_CONTROL_CONFIG.valid(),
                  "WHEEL_CONTROL_CONFIG: check COMMAND_TIMEOUT_MS and UPDATE_RATE_CONTROL");

    // Connects the wheel control to the rest of the firmware. The subscriber passes each valid
    // wheel command to wheel_control.receiveCommand(), and loop() calls wheel_control.step()
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

    // Logs the encoder readings and the PID state at UPDATE_RATE_DEBUG, while the parameter
    // /diffbot/debug/base_controller is true
    void publishDebugIfDue(uint32_t now_ms)
    {
        if (base_controller.debug())
        {
            if (now_ms - base_controller.lastUpdateTime().debug
                >= base_controller.publishRate().period().debug_ * 1000.0)
            {
                base_controller.printDebug();
                base_controller.lastUpdateTime().debug = now_ms;
            }
        }
    }
}


void setup()
{
    // Register the wheel subscriber and the IMU publisher before topic negotiation, the handshake
    // between the Teensy and the robot's single-board computer (SBC):
    // 1. BaseController::setup() initializes the Teensy's node handle, registers the remaining
    //    topics and calls spinOnce() until the node handle reports a connection.
    // 2. On the SBC, serial_node.py opens the USB serial device and requests the Teensy's topic
    //    descriptions. serial_node.py is part of rosserial_python, not of this repository.
    // 3. spinOnce() handles that request and sends the registered publishers and subscribers.
    // 4. serial_node.py creates the corresponding ROS endpoints on the SBC. Its ROS subscriber
    //    forwards wheel commands to the Teensy, where spinOnce() dispatches the callback.
    // Registering a new topic later doesn't notify the SBC until another topic negotiation.
    // The two programs start independently. BaseController::init() also lets callbacks run
    // through spinOnce() while waiting for parameter replies.
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

    publishDebugIfDue(now_ms);

    // Parse incoming rosserial bytes and dispatch complete messages after the control step.
    // The next loop pass reads now_ms after these callbacks. A callback timestamp later than
    // now_ms would otherwise make unsigned subtraction report an age near 49.7 days
    nh.spinOnce();
}
