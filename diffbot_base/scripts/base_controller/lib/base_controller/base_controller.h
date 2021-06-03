/*
 * Author: Franz Pucher
 */

#ifndef DIFFBOT_BASE_CONTROLLER_H
#define DIFFBOT_BASE_CONTROLLER_H


#include <ros.h>
//#include <std_msgs/Int32.h>
#include <diffbot_msgs/EncodersStamped.h>
#include <diffbot_msgs/WheelsCmdStamped.h>
#include <std_msgs/Empty.h>

#include "diffbot_base_config.h"
#include "encoder.h"
#include "adafruit_feather_wing/adafruit_feather_wing.h"


namespace diffbot {


    template <typename TMotorController, typename TMotorDriver>
    class BaseController
    {
    public:


        BaseController(ros::NodeHandle &nh, TMotorController* motor_controller_left, TMotorController* motor_controller_right);

        struct PublishRate
        {
            double imu_;
            double command_;
            double debug_;

            struct Period
            {
                double imu_;
                double command_;
                double debug_;

                inline Period(double imu_frequency, double command_frequency, double debug_frequency)
                    : imu_(1.0 / imu_frequency)
                    , command_(1.0 / command_frequency)
                    , debug_(1.0 / debug_frequency) {};
            } period_;

            inline Period& period() { return period_; };

            inline PublishRate(double imu_frequency,
                               double command_frequency,
                               double debug_frequency)
                : imu_(imu_frequency)
                , command_(command_frequency)
                , debug_(debug_frequency)
                , period_(imu_frequency, command_frequency, debug_frequency) {};
        } publish_rate_;

        inline int period(double frequency) { return 1 / frequency; };

        inline PublishRate& publishRate() { return publish_rate_; };

        struct LastUpdateTime
        {
            ros::Time command;
            ros::Time control;
            ros::Time imu;
            ros::Time debug;

            inline LastUpdateTime(ros::Time start)
                : command(start.toSec(), start.toNsec())
                , control(start.toSec(), start.toNsec())
                , imu(start.toSec(), start.toNsec())
                , debug(start.toSec(), start.toNsec()) {};
        } last_update_time_;

        inline LastUpdateTime& lastUpdateTime() { return last_update_time_; };


        inline bool debug() { return debug_; };

        void setup();
        void init();
        void eStop();
        void read();
        void write();
        void printDebug();


        void commandCallback(const diffbot_msgs::WheelsCmdStamped& cmd_msg);
        void resetEncodersCallback(const std_msgs::Empty& reset);

    private:
        ros::NodeHandle& nh_;

        // constants
        float wheel_radius_ = 0.0;
        float max_linear_velocity_ = 0.0;
        float max_angular_velocity_ = 0.0;

        // Encoder setup
        // Change these pin numbers to the pins connected to your encoder.
        //   Best Performance: both pins have interrupt capability
        //   Good Performance: only the first pin has interrupt capability
        //   Low Performance:  neither pin has interrupt capability
        // avoid using pins with LEDs attached
        diffbot::Encoder encoder_left_;
        diffbot::Encoder encoder_right_;

        unsigned long encoder_resolution_;

        ros::Subscriber<std_msgs::Empty, BaseController<TMotorController, TMotorDriver>> sub_reset_encoders_;

        // ROS Publisher setup to publish left and right encoder ticks
        // This uses the custom encoder ticks message that defines an array of two integers
        diffbot_msgs::EncodersStamped encoder_msg_;
        ros::Publisher pub_encoders_;


        MotorControllerIntf<TMotorDriver>* p_motor_controller_right_;
        MotorControllerIntf<TMotorDriver>* p_motor_controller_left_;

        ros::Subscriber<diffbot_msgs::WheelsCmdStamped, BaseController<TMotorController, TMotorDriver>> sub_wheel_cmd_velocities_;
        float wheel_cmd_velocity_left_ = 0.0;
        float wheel_cmd_velocity_right_ = 0.0;

        // DEBUG
        bool debug_;
    };


}

template <typename TMotorController, typename TMotorDriver>
using BC = diffbot::BaseController<TMotorController, TMotorDriver>;


template <typename TMotorController, typename TMotorDriver>
diffbot::BaseController<TMotorController, TMotorDriver>
    ::BaseController(ros::NodeHandle &nh, TMotorController* motor_controller_left, TMotorController* motor_controller_right)
    : nh_(nh)
    , encoder_left_(ENCODER_LEFT_H1, ENCODER_LEFT_H2, ENCODER_RESOLUTION)
    , encoder_right_(ENCODER_RIGHT_H1, ENCODER_RIGHT_H2, ENCODER_RESOLUTION)
    , sub_reset_encoders_("reset", &BC<TMotorController, TMotorDriver>::resetEncodersCallback, this)
    , pub_encoders_("encoder_ticks", &encoder_msg_)
    , sub_wheel_cmd_velocities_("wheel_cmd_velocities", &BC<TMotorController, TMotorDriver>::commandCallback, this)
    , last_update_time_(nh.now())
    , publish_rate_(PUBLISH_RATE_IMU, PUBLISH_RATE_COMMAND, PUBLISH_RATE_DEBUG)
{
    p_motor_controller_left_ = motor_controller_left;
    p_motor_controller_right_ = motor_controller_right;
}


template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::setup()
{
    nh_.initNode();
    nh_.advertise(pub_encoders_);
    nh_.subscribe(sub_wheel_cmd_velocities_);
    nh_.subscribe(sub_reset_encoders_);

    while (!nh_.connected())
    {
        nh_.spinOnce();
    }
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::init()
{
    nh_.loginfo("Get Parameters from Parameter Server");
    nh_.getParam("/diffbot/encoder_resolution", &this->encoder_resolution_);
    String log_msg = String("/diffbot/encoder_resolution: ") + String(encoder_resolution_);
    nh_.loginfo(log_msg.c_str());
    nh_.getParam("/diffbot/mobile_base_controller/wheel_radius", &wheel_radius_);
    log_msg = String("/diffbot/mobile_base_controller/wheel_radius: ") + String(wheel_radius_);
    nh_.loginfo(log_msg.c_str());
    nh_.getParam("/diffbot/mobile_base_controller/linear/x/max_velocity", &max_linear_velocity_);
    log_msg = String("/diffbot/mobile_base_controller/linear/x/max_velocity: ") + String(max_linear_velocity_);
    nh_.loginfo(log_msg.c_str());

    nh_.loginfo("Initialize DiffBot Wheel Encoders");
    encoder_left_.resolution(encoder_resolution_);
    encoder_right_.resolution(encoder_resolution_);

    std_msgs::Empty reset;
    this->resetEncodersCallback(reset);
    delay(1);

    max_angular_velocity_ = max_linear_velocity_ / wheel_radius_;

    delay(1000);
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::commandCallback(const diffbot_msgs::WheelsCmdStamped& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    wheel_cmd_velocity_left_ = cmd_msg.wheels_cmd.angular_velocities.joint[0];
    wheel_cmd_velocity_right_ = cmd_msg.wheels_cmd.angular_velocities.joint[1];

    lastUpdateTime().command = nh_.now();
}

// ROS Subscriber setup to reset both encoders to zero
template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::resetEncodersCallback(const std_msgs::Empty& reset)
{
    //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
    // reset both back to zero.
    this->encoder_left_.write(0);
    this->encoder_right_.write(0);
    this->nh_.loginfo("Reset both wheel encoders to zero");
}


template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::read()
{
    //get the current speed of each motor from the encoder ticks
    long new_left, new_right;
    new_left = encoder_left_.read();
    new_right = encoder_right_.read();

    encoder_msg_.encoders.ticks[0] = new_left;
    encoder_msg_.encoders.ticks[1] = new_right;
    pub_encoders_.publish(&encoder_msg_);

    // TODO publish low level velocities and compare with high level
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::write()
{
    // https://www.arduino.cc/reference/en/language/functions/math/map/
    // map(value, fromLow, fromHigh, toLow, toHigh)
    // Map angular wheel joint velocity to motor cmd
    // TODO get rosparam linear max_velocity and convert to max rotational max velocity
    int motor_cmd_left = map(wheel_cmd_velocity_left_, -max_angular_velocity_, max_angular_velocity_, -255, 255);

    // TODO compute PID output
    //the value sent to the motor driver is calculated by the PID based on the error between commanded RPM vs measured RPM
    //the calculated PID ouput value is is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // float measured_angular_velocity_left = encoderLeft.getRPM();
    //motor_pid_left.compute(g_wheel_cmd_velocity_left, measured_angular_velocity_left));
    p_motor_controller_left_->setSpeed(motor_cmd_left);

    int motor_cmd_right = map(wheel_cmd_velocity_right_, -max_angular_velocity_, max_angular_velocity_, -255, 255);
    p_motor_controller_right_->setSpeed(motor_cmd_right);
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::eStop()
{
    wheel_cmd_velocity_left_ = 0;
    wheel_cmd_velocity_right_ = 0;
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::printDebug()
{

}


#endif // DIFFBOT_BASE_CONTROLLER_H