#include <ros.h>

#include "diffbot_base_config.h"
#include "base_controller.h"
#include "adafruit_feather_wing/adafruit_feather_wing.h"


ros::NodeHandle nh;

using namespace diffbot;

AdafruitMotorController motor_controller_right = AdafruitMotorController(3);
AdafruitMotorController motor_controller_left = AdafruitMotorController(4);

diffbot::BaseController<AdafruitMotorController, Adafruit_MotorShield> base_controller(nh, &motor_controller_left, &motor_controller_right);


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

    //this block drives the robot based on defined rate
    ros::Duration command_dt = nh.now() - base_controller.lastUpdateTime().control;
    if (command_dt.toSec() >= ros::Duration(1.0 / base_controller.publishRate().command_, 0).toSec())
    {
        base_controller.read();
        base_controller.write();
        base_controller.lastUpdateTime().control = nh.now();
    }

    //this block stops the motor when no command is received
    command_dt = nh.now() - base_controller.lastUpdateTime().command;
    if (command_dt.toSec() >= ros::Duration(400, 0).toSec())
    {
        base_controller.eStop();
    }

    //this block publishes the IMU data based on defined rate
    ros::Duration imu_dt = nh.now() - base_controller.lastUpdateTime().imu;
    if (imu_dt.toSec() >= base_controller.publishRate().period().imu_)
    {
        //sanity check if the IMU is connected
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
        base_controller.lastUpdateTime().imu = nh.now();
    }

    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(base_controller.debug())
    {
        ros::Duration debug_dt = nh.now() - base_controller.lastUpdateTime().debug;
        if (debug_dt.toSec() >= base_controller.publishRate().period().debug_)
        {
            base_controller.printDebug();
            base_controller.lastUpdateTime().debug = nh.now();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
}