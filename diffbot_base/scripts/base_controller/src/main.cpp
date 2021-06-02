#include <Arduino.h>

#include <ros.h>

#include "diffbot_base_config.h"
#include "base_controller.h"
#include "adafruit_feather_wing/adafruit_feather_wing.h"


ros::NodeHandle nh;

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
    if ((millis() - base_controller.last_update_time().control) >= (1000 / base_controller.publish_rate_.command_))
    {
        base_controller.read();
        base_controller.write();
        base_controller.last_update_time().control = millis();
    }

    //this block stops the motor when no command is received
    if ((millis() - base_controller.last_update_time().command) >= 400)
    {
        base_controller.eStop();
    }

    //this block publishes the IMU data based on defined rate
    if ((millis() - base_controller.last_update_time().imu) >= (1000 / base_controller.publish_rate_.imu_))
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
        base_controller.last_update_time().imu = millis();
    }

    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(base_controller.debug())
    {
        if ((millis() - base_controller.last_update_time().debug) >= (1000 / base_controller.publish_rate_.debug_))
        {
            base_controller.printDebug();
            base_controller.last_update_time().debug = millis();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
}