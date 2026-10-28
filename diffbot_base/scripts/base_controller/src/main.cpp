#include <ros.h>

#include "diffbot_base_config.h"
#include "base_controller.h"
#include "adafruit_feather_wing/adafruit_feather_wing.h"


ros::NodeHandle nh;

using namespace diffbot;

AdafruitMotorController motor_controller_right = AdafruitMotorController(MOTOR_RIGHT);
AdafruitMotorController motor_controller_left = AdafruitMotorController(MOTOR_LEFT);

BaseController<AdafruitMotorController, Adafruit_MotorShield> base_controller(nh, &motor_controller_left, &motor_controller_right);


void setup()
{
    base_controller.setup();
    base_controller.init();

    nh.loginfo("Initialize DiffBot Motor Controllers");
    motor_controller_left.begin();
    motor_controller_right.begin();
    nh.loginfo("Setup finished");
}


void loop()
{
    static bool imu_is_initialized;

    // The times are the Teensy's millis(), which only counts up; ROS time can jump (see
    // BaseController::LastUpdateTime). Unsigned differences stay right when millis() wraps
    uint32_t now_ms = millis();

    // The main control loop for the base_conroller.
    // This block drives the robot based on a defined control rate
    if (now_ms - base_controller.lastUpdateTime().control >= 1000.0 / base_controller.publishRate().control_)
    {
        base_controller.read();
        base_controller.write();
        base_controller.lastUpdateTime().control = now_ms;
    }

    // This block stops the motor when no wheel command is received
    // from the high level hardware_interface::RobotHW
    if (now_ms - base_controller.lastUpdateTime().command_received >= E_STOP_COMMAND_RECEIVED_DURATION * 1000UL)
    {
        nh.logwarn("Emergency STOP");
        base_controller.eStop();
    }

    // This block publishes the IMU data based on a defined imu rate
    if (now_ms - base_controller.lastUpdateTime().imu >= base_controller.publishRate().period().imu_ * 1000.0)
    {
        // Sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            //imu_is_initialized = initIMU();
            if(imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            //publishIMU();
        }
        base_controller.lastUpdateTime().imu = now_ms;
    }

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