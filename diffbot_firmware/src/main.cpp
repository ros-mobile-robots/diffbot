#include <ros.h>
#include <Wire.h>

#include "diffbot_base_config.h"
#include "base_controller.h"
#include "command_timeout.h"
#include "drive_step.h"
#include "adafruit_feather_wing/adafruit_feather_wing.h"
#include "bno055.h"
#include "imu_publisher.h"
#include "synced_node_handle.h"
#include "wire_bus.h"


// The node handle also tells when its clock is synchronized with the host (synced_node_handle.h)
typedef diffbot::SyncedNodeHandleT<ros::NodeHandle> NodeHandle;
NodeHandle nh;

using namespace diffbot;

AdafruitMotorController motor_controller_right = AdafruitMotorController(MOTOR_RIGHT);
AdafruitMotorController motor_controller_left = AdafruitMotorController(MOTOR_LEFT);

BaseController<AdafruitMotorController, Adafruit_MotorShield> base_controller(nh, &motor_controller_left, &motor_controller_right);

// Stops the motors when the wheel commands from the robot's computer stop arriving (loop())
CommandTimeout command_timeout(COMMAND_TIMEOUT_MS);

// The Arduino clock for the IMU publisher, which the unit tests replace with a fake one
struct ArduinoClock
{
    static uint32_t millis() { return ::millis(); }
    static uint32_t micros() { return ::micros(); }
};

// The IMU, on the same I2C bus as the motor driver. It publishes imu/data_raw, which rosserial
// puts in its node's namespace (/diffbot/imu/data_raw). The robot drives without an IMU too;
// IMU_ENABLED 0 in diffbot_base_config.h turns it off for a robot without one.
WireBusT<TwoWire> wire_bus(Wire);
Bno055 imu(wire_bus, IMU_BNO055_ADDRESS);
ImuPublisher<NodeHandle, ArduinoClock> imu_publisher(nh, imu);


void setup()
{
#if IMU_ENABLED
    // Advertise before base_controller.setup() connects: rosserial announces the topics then
    imu_publisher.advertise();
#endif
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
    uint32_t now_ms = millis();

    // Without a wheel command for COMMAND_TIMEOUT_MS, stop the motors and reset the PIDs, then log
    // once. The control step drives the motors again after the next command (lib/drive_step).
    driveStep(base_controller, command_timeout, now_ms,
              1000.0 / base_controller.publishRate().control_,
              []() { nh.logwarn("Wheel command timeout: motors stopped"); });

#if IMU_ENABLED
    imu_publisher.update();
#endif

    // This block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(base_controller.debug())
    {
        if (now_ms - base_controller.lastUpdateTime().debug >= base_controller.publishRate().period().debug_ * 1000.0)
        {
            base_controller.printDebug();
            base_controller.lastUpdateTime().debug = now_ms;
        }
    }
    // Call all the callbacks waiting to be called
    nh.spinOnce();
}