#include <ros.h>
#include <Wire.h>

#include "diffbot_base_config.h"
#include "base_controller.h"
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

// The IMU's objects, private to this file
namespace
{
    // The Arduino clock for the IMU publisher, which the unit tests replace with a fake one
    struct ArduinoClock
    {
        static uint32_t millis() { return ::millis(); }
        static uint32_t micros() { return ::micros(); }
    };

    // The IMU, on the same I2C bus as the motor driver. It publishes imu/data_raw, which rosserial
    // puts in its node's namespace (/diffbot/imu/data_raw). The robot drives without an IMU too.
    // IMU_PUBLISHER_CONFIG in diffbot_base_config.h turns it off for a robot without one.
    WireBusT<TwoWire> wire_bus(Wire);
    Bno055 imu(wire_bus, IMU_BNO055_ADDRESS);
    ImuPublisher<NodeHandle, ArduinoClock> imu_publisher(nh, imu, IMU_PUBLISHER_CONFIG);
}


void setup()
{
    // Advertise before base_controller.setup() connects: rosserial announces the topics then
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
    uint32_t now_ms = millis();

    // The main control loop for the base_conroller.
    // This block drives the robot based on a defined control rate
    if (now_ms - base_controller.lastUpdateTime().control >= 1000.0 / base_controller.publishRate().control_)
    {
        base_controller.read();
        base_controller.write();
        base_controller.lastUpdateTime().control = now_ms;
    }

    // After 5 s without a wheel command, set both target velocities to zero.
    // eStop() does not directly stop the motors or reset the PIDs.
    if (now_ms - base_controller.lastUpdateTime().command_received >= E_STOP_COMMAND_RECEIVED_DURATION * 1000UL)
    {
        nh.logwarn("Wheel command timeout: targets set to zero");
        base_controller.eStop();
    }

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
    // Call all the callbacks waiting to be called
    nh.spinOnce();
}