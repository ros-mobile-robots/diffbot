/*
 * Author: Franz Pucher
 */

#ifndef DIFFBOT_BASE_CONTROLLER_H
#define DIFFBOT_BASE_CONTROLLER_H


#include <ros.h>
#include <diffbot_msgs/EncodersStamped.h>
#include <diffbot_msgs/WheelsCmdStamped.h>
#include <diffbot_msgs/AngularVelocities.h>
#include <diffbot_msgs/PIDStamped.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include "diffbot_base_config.h"
#include "encoder_diffbot.h"
#include "adafruit_feather_wing/adafruit_feather_wing.h"
#include "pid.h"


namespace diffbot {

    /** \brief Communicates with the high level hardware_interface::RobotHW and interacts with
     *         the robot hardware sensors (e.g. encoders) and actuators (e.g. motor driver).
     * 
     * The BaseController communicates with the high level interface diffbot_base::hardware_interface::RobotHW
     * using ROS publishers and subscribers. In the main loop (see main.cpp) this class is 
     * operated at specific update rates \ref update_rate_ for different sensors and acutators.
     * There exist three update rates:
     * - control
     * - imu
     * - debug
     * 
     * The control update rate is used to read from the encoders and write computed pwm signals to the motors
     * at a certain rate. This is important to avoid running into syncronization errors because 
     * of too many running calculations.
     * Similarly the imu update rate reads the latest inertial measurements at a specific rate. 
     * To keep track when a new update is needed, the \ref last_update_time_ member is used, 
     * which stores the time stamps when the last update happened. 
     * Additionally the \ref LastUpdateTime::command_received time stamp
     * is used to check for connection losses to the high level hardware interface, which would then
     * stop the motors (see \ref eStop), if no command message was received for a specified amount of time.
     * 
     * The BaseController subscribes to the target wheel command angular velocities
     * (\ref diffbot_msgs::WheelsCmdStamped on the "wheel_cmd_velocities" topic) with the 
     * \ref sub_wheel_cmd_velocities_ and keeps pointers to both motors 
     * in \ref p_motor_controller_right_ and \ref p_motor_controller_left_ using a generic motor driver agnostic interface 
     * \ref diffbot::MotorControllerIntf. Each time a new \ref diffbot_msgs::WheelsCmdStamped is received on the
     * "wheel_cmd_velocities" topic the \ref commandCallback is called and the two target velocities \ref wheel_cmd_velocity_left_
     * and \ref wheel_cmd_velocity_right_ for each wheel are updated.
     * 
     * To measure the angular wheel positions (absolut) and the angular velocities, 
     * two \ref diffbot::Encoder objects (\ref encoder_left_ \ref encoder_right_)
     * are used to \ref read() the diffbot::JointStates, stored in \ref joint_state_left_ and \ref joint_state_left_.
     * After reading the latest states (position and velocity), the values are published with \ref pub_measured_joint_states_ on 
     * "measured_joint_states" topic of type \ref sensor_msgs::JointState. The \ref diffbot_base::hardware_interface::RobotHW
     * subscribes to these joint states and passes them on to the \ref diff_drive_controller.
     * Also inside the \ref read() method, the current encoder ticks read from the \ref encoder_left_ and \ref encoder_right_
     * and stored in \ref ticks_left_ and \ref ticks_right_.
     * 
     * The measured angular velocities are important to compute the pwm signals for each motor using two separate
     * PID controllers (\ref motor_pid_left_ and \ref motor_pid_right_), which calculate 
     * the error between measured and commanded angular wheel velocity for each wheel joint.
     * 
     * For initialization the following parameters are read from the ROS parameter server.
     * - /diffbot/encoder_resolution
     * - /diffbot/mobile_base_controller/wheel_radius
     * - /diffbot/mobile_base_controller/linear/x/max_velocity
     * - /diffbot/debug/base_controller
     * 
     * @tparam TMotorController 
     * @tparam TMotorDriver 
     */
    template <typename TMotorController, typename TMotorDriver>
    class BaseController
    {
    public:

        /**
         * @brief Construct a new Base Controller object using two generic motor controllers
         * 
         * Requires two initialized generic motor controllers, which are defined in a parent scope (e.g. main.cpp)
         * The two motor controllers for the left and right motor are kept generic.
         * The only requirement is to implement the \ref diffbot::MotorControllerIntf,
         * which is composed of the motor driver and has therefore the ability to control one motor.
         * 
         * @param nh Reference to the global ROS node handle
         * @param motor_controller_left Pointer to the generic motor controller for the left motor
         * @param motor_controller_right Pointer to the generic motor controller for the right motor
         */
        BaseController(ros::NodeHandle &nh, TMotorController* motor_controller_left, TMotorController* motor_controller_right);

        /**
         * @brief Stores update rate frequencies (Hz) for the main control loop, (optinal) imu and debug output.
         * 
         * The UpdateRate contains a nested strcut \ref period_ T, which define the inverse of the rate
         * T = 1/f HZ  in seconds. The period (and indirectly the rate) is used together with 
         * \ref last_update_time_ to compare if enought time elapsed for a new update. 
         * The comparison is done in the main loop (see, main.cpp).
         */
        struct UpdateRate
        {
            double imu_;
            double control_;
            double debug_;

            /**
             * @brief Inverse of the update rates for control, (optional) imu and debug output.
             * 
             * See \ref UpdateRate for more details.
             */
            struct Period
            {
                double imu_;
                double control_;
                double debug_;

                inline Period(double imu_frequency, double control_frequency, double debug_frequency)
                    : imu_(1.0 / imu_frequency)
                    , control_(1.0 / control_frequency)
                    , debug_(1.0 / debug_frequency) {};
            } period_;

            inline Period& period() { return period_; };

            /**
             * @brief Construct a new Update Rate object.
             * 
             * Using the frequency parameters \p imu_frequency,
             * \p control_frequency and \p debug_frequency the corresponding
             * periods of \ref period_ are initilized.
             * 
             * @param imu_frequency Defines how often the imu is read and published.
             * @param control_frequency Defines how ofthen the control block is run (reading encoders and writing motor commands).
             * @param debug_frequency Defines how often debug messages are output.
             */
            inline UpdateRate(double imu_frequency,
                               double control_frequency,
                               double debug_frequency)
                : imu_(imu_frequency)
                , control_(control_frequency)
                , debug_(debug_frequency)
                , period_(imu_frequency, control_frequency, debug_frequency) {};
        } update_rate_;

        /**
         * @brief Calculates the period (s) from a given \p frequency (Hz).
         * 
         * @param frequency Input frequency (Hz) to be converted to period.
         * @return int period in seconds.
         */
        inline int period(double frequency) { return 1 / frequency; };

        inline UpdateRate& publishRate() { return update_rate_; };

        /**
         * @brief Keeps track of the last update times.
         * 
         * Inside the main.cpp the members of this \ref last_update_time_
         * are updated everytime the corresponding event happens. 
         * This is used together with the \ref update_rate_ to check if
         * enough time elapsed for a new update. For example reading new
         * values from the encoders and writing the received motor commands.
         */
        struct LastUpdateTime
        {
            // Time when the last angular wheel command velocity was received.
            ros::Time command_received;
            // Time when the last control update happened.
            ros::Time control;
            // Time when the last imu update took place.
            ros::Time imu;
            // Time when the last debug message was logged.
            ros::Time debug;

            /**
             * @brief Construct a new Last Update Time object
             * 
             * This object is initialized with the current time using
             * the \ref ros::NodeHandle::now() method.
             * 
             * @param start Start time of the program
             */
            inline LastUpdateTime(ros::Time start)
                : command_received(start.toSec(), start.toNsec())
                , control(start.toSec(), start.toNsec())
                , imu(start.toSec(), start.toNsec())
                , debug(start.toSec(), start.toNsec()) {};
        } last_update_time_;

        /**
         * @brief Returns a reference to \ref last_update_times_.
         * 
         * Used inside the main loop inside main.cpp to compare with the
         * current time if an update of a specific function (command_received, control, imu, debug)
         * is needed.
         * 
         * @return LastUpdateTime& The pervious update times.
         */
        inline LastUpdateTime& lastUpdateTime() { return last_update_time_; };


        /**
         * @brief Getter if the firmware should log debug output.
         * 
         * @return true 
         * @return false 
         */
        inline bool debug() { return debug_; };

        /**
         * @brief Initializes the main node handle and setup publisher and subscriber.
         * 
         * Waits until the connection to the ros master is established.
         */
        void setup();

        /**
         * @brief Reads parameters from the parameter server
         * 
         * - Get Parameters from Parameter Server
         *   - /diffbot/encoder_resolution
         *   - /diffbot/mobile_base_controller/wheel_radius
         *   - /diffbot/mobile_base_controller/linear/x/max_velocity
         *   - /diffbot/debug/base_controller
         * - Initialize DiffBot Wheel Encoders
         * - Reset both wheel encoders tick count to zero
         * - Initialize the \ref max_angular_velocity_ from the read \ref max_linear_velocity_ and \ref wheel_radius_
         */
        void init();

        /**
         * @brief Stops the motors, in case no wheel commands are received over a longer time period.
         * 
         * Sets the \ref wheel_cmd_velocity_left_ and \ref wheel_cmd_velocity_right_ to zero.
         * This method is called when the \ref commandCallback wasn't called within the 
         * \ref LastUpdateTime::command_received period on the /diffbot/wheel_cmd_velocities topic.
         * 
         */
        void eStop();

        /**
         * @brief Reads the current encoder tick counts and joint states (angular position (rad) and angular velocity (rad/s) 
         *        from both encoders left \ref encoder_left_ and right \ref encoder_right_ 
         *        and publishes sensor_msgs::JointState on the "measured_joint_states" topic,
         *        using the \ref pub_measured_joint_states_.
         *
         */
        void read();

        /**
         * @brief Uses the PIDs to compute capped PWM signals for the left and right motor.
         * 
         * The values for both motors sent to the motor driver are calculated by the two PIDs
         * \ref pid_motor_left_ and \ref pid_motor_right_, based on the error between 
         * commanded angular velocity vs measured angular velocity
         * (e.g. diffbot::PID::error_ = \ref wheel_cmd_velocity_left_ - \ref measured_angular_velocity_left_)
         * 
         * The calculated PID ouput values are capped at -/+ MAX_RPM to prevent the PID from having too much error
         * The computed and capped PID values are then set using the \ref diffbot::MotorControllerIntf::setSpeed method
         * for the left and right motors \ref p_motor_controller_left_ and \ref p_motor_controller_right_.
         * 
         */
        void write();
        void printDebug();

        /**
         * @brief Callback method when a new \ref diffbot_msgs::WheelsCmdStamped is received on the wheel_cmd_velocities topic.
         * 
         * Callback method every time the angular wheel commands for each wheel joint are received from 'wheel_cmd_velocities' topic.
         * This topic is published from the high level \ref hardware_interface::RobotHW::write() method.
         * 
         * This callback method receives \ref diffbot_msgs::WheelsCmdStamped message object (\p cmd_msg) consisting of
         * \ref diffbot_msgs::AngularVelocities, where commanded wheel joints for the left and right wheel are stored.
         * After receiving the message the \ref wheel_cmd_velocity_left_ and \ref wheel_cmd_velocity_right_ members are updated,
         * which are used in the next \ref write() of the control loop.
         * 
         * \code
         * wheel_cmd_velocity_left_ = cmd_msg.wheels_cmd.angular_velocities.joint[0];
         * wheel_cmd_velocity_right_ = cmd_msg.wheels_cmd.angular_velocities.joint[1];
         * \endcode
         * 
         * In this method the \ref lastUpdateTime::command_received time stamp is set to the current time.
         * This is used for the eStop functionality. In case no diffbot_msgs::WheelsCmdStamped messages are 
         * received on the wheel_cmd_velocities topic, the eStop method is called (see main loop in main.cpp)
         * 
         * @param cmd_msg Message containing the commanded wheel velocities.
         */
        void commandCallback(const diffbot_msgs::WheelsCmdStamped& cmd_msg);

        /**
         * @brief Callback method to reset both encoder's tick count to zero.
         * 
         * For initializing the the BaseController the encoder tick counts are set back to zero.
         * Every time the diffbot_bringup/launch/bringup.launch is launched, the high level
         * hardware_interface::RobotHW publishes an empty message on the /reset topic,
         * which invokes this callback and sets the encoders to zero.
         * 
         * @param reset_msg empty message (unused)
         */
        void resetEncodersCallback(const std_msgs::Empty& reset_msg);

        /**
         * @brief Callback method to update the PID constants for the left motor.
         *
         * Publish to pid_left topic to update the PID constants of the left motor.
         *
         * @param pid_msg message containing the new PID constants
         */
        void pidLeftCallback(const diffbot_msgs::PIDStamped& pid_msg);

        /**
         * @brief Callback method to update the PID constants for the right motor.
         *
         * Publish to pid_right topic to update the PID constants of the right motor.
         *
         * @param pid_msg message containing the new PID constants
         */
        void pidRightCallback(const diffbot_msgs::PIDStamped& pid_msg);

    private:
        // Reference to global node handle from main.cpp
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
        long ticks_left_ = 0, ticks_right_ = 0;

        // Measured left and right joint states (angular position (rad) and angular velocity (rad/s))
        diffbot::JointState joint_state_left_, joint_state_right_;

        unsigned long encoder_resolution_;

        ros::Subscriber<std_msgs::Empty, BaseController<TMotorController, TMotorDriver>> sub_reset_encoders_;

        // ROS Publisher setup to publish left and right encoder ticks
        // This uses the custom encoder ticks message that defines an array of two integers
        diffbot_msgs::EncodersStamped encoder_msg_;
        ros::Publisher pub_encoders_;

        sensor_msgs::JointState msg_measured_joint_states_;
        ros::Publisher pub_measured_joint_states_;

        MotorControllerIntf<TMotorDriver>* p_motor_controller_right_;
        MotorControllerIntf<TMotorDriver>* p_motor_controller_left_;

        ros::Subscriber<diffbot_msgs::WheelsCmdStamped, BaseController<TMotorController, TMotorDriver>> sub_wheel_cmd_velocities_;
        float wheel_cmd_velocity_left_ = 0.0;
        float wheel_cmd_velocity_right_ = 0.0;

        int motor_cmd_left_ = 0;
        int motor_cmd_right_ = 0;

        ros::Subscriber<diffbot_msgs::PIDStamped, BaseController<TMotorController, TMotorDriver>> sub_pid_left_;
        ros::Subscriber<diffbot_msgs::PIDStamped, BaseController<TMotorController, TMotorDriver>> sub_pid_right_;
        PID motor_pid_left_;
        PID motor_pid_right_;

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
    , encoder_left_(nh, ENCODER_LEFT_H1, ENCODER_LEFT_H2, ENCODER_RESOLUTION)
    , encoder_right_(nh, ENCODER_RIGHT_H1, ENCODER_RIGHT_H2, ENCODER_RESOLUTION)
    , sub_reset_encoders_("reset", &BC<TMotorController, TMotorDriver>::resetEncodersCallback, this)
    , pub_encoders_("encoder_ticks", &encoder_msg_)
    , pub_measured_joint_states_("measured_joint_states", &msg_measured_joint_states_)
    , sub_wheel_cmd_velocities_("wheel_cmd_velocities", &BC<TMotorController, TMotorDriver>::commandCallback, this)
    , last_update_time_(nh.now())
    , update_rate_(UPDATE_RATE_IMU, UPDATE_RATE_CONTROL, UPDATE_RATE_DEBUG)
    , sub_pid_left_("pid_left", &BC<TMotorController, TMotorDriver>::pidLeftCallback, this)
    , sub_pid_right_("pid_right", &BC<TMotorController, TMotorDriver>::pidRightCallback, this)
    , motor_pid_left_(PWM_MIN, PWM_MAX, K_P, K_I, K_D)
    , motor_pid_right_(PWM_MIN, PWM_MAX, K_P, K_I, K_D)
{
    p_motor_controller_left_ = motor_controller_left;
    p_motor_controller_right_ = motor_controller_right;
}


template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::setup()
{
    nh_.initNode();
    nh_.advertise(pub_encoders_);

    // msg_measured_joint_states_ is of type sensor_msgs::JointState
    // which contains float[] joint arrays of undefined size.
    // For rosserial to work it is required to reserve the memory using malloc
    // and setting the *_length member appropriately.
    // http://wiki.ros.org/rosserial/Overview/Limitations#Arrays
    msg_measured_joint_states_.position = (float*)malloc(sizeof(float) * 2);
    msg_measured_joint_states_.position_length = 2;
    msg_measured_joint_states_.velocity = (float*)malloc(sizeof(float) * 2);
    msg_measured_joint_states_.velocity_length = 2;
    nh_.advertise(pub_measured_joint_states_);

    nh_.subscribe(sub_wheel_cmd_velocities_);
    nh_.subscribe(sub_reset_encoders_);

    nh_.subscribe(sub_pid_left_);
    nh_.subscribe(sub_pid_right_);

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
    nh_.getParam("/diffbot/debug/base_controller", &debug_);
    log_msg = String("/diffbot/debug/base_controller: ") + String(debug_);
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
    // Callback function every time the angular wheel commands for each wheel joint are received from 'wheel_cmd_velocities' topic
    // This callback function receives diffbot_msgs::WheelsCmdStamped message object
    // where diffbot_msgs::AngularVelocities for both joints are stored
    wheel_cmd_velocity_left_ = cmd_msg.wheels_cmd.angular_velocities.joint[0];
    wheel_cmd_velocity_right_ = cmd_msg.wheels_cmd.angular_velocities.joint[1];

    // Used for the eStop. In case no diffbot_msgs::WheelsCmdStamped messages are received on the wheel_cmd_velocities 
    // topic, the eStop method is called (see main loop in main.cpp)
    lastUpdateTime().command_received = nh_.now();
}

// ROS Subscriber setup to reset both encoders to zero
template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::resetEncodersCallback(const std_msgs::Empty& reset_msg)
{
    // reset both back to zero.
    this->encoder_left_.write(0);
    this->encoder_right_.write(0);
    this->nh_.loginfo("Reset both wheel encoders to zero");
}


template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::pidLeftCallback(const diffbot_msgs::PIDStamped& pid_msg)
{
    // Callback function to update the pid values of the left motor
    // This callback function receives diffbot_msgs::PID message object
    // where diffbot_msgs::PID kp, ki, kd for one pid controller is stored
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::pidRightCallback(const diffbot_msgs::PIDStamped& pid_msg)
{
    // Callback function to update the pid values of the left motor
    // This callback function receives diffbot_msgs::PID message object
    // where diffbot_msgs::PID kp, ki, kd for one pid controller is stored
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
    motor_pid_left_.updateConstants(pid_msg.pid.kp, pid_msg.pid.ki, pid_msg.pid.kd);
}


template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::read()
{
    joint_state_left_ = encoder_left_.jointState();
    joint_state_right_ = encoder_right_.jointState();

    msg_measured_joint_states_.position[0] = joint_state_left_.angular_position_;
    msg_measured_joint_states_.position[1] = joint_state_right_.angular_position_;

    msg_measured_joint_states_.velocity[0] = joint_state_left_.angular_velocity_;
    msg_measured_joint_states_.velocity[1] = joint_state_right_.angular_velocity_;
    pub_measured_joint_states_.publish(&msg_measured_joint_states_);

    // get the current tick count of each encoder
    ticks_left_ = encoder_left_.read();
    ticks_right_ = encoder_right_.read();

    encoder_msg_.encoders.ticks[0] = ticks_left_;
    encoder_msg_.encoders.ticks[1] = ticks_right_;
    // Avoid having too many publishers
    // Otherwise error like 'wrong checksum for topic id and msg'
    // and 'Write timeout: Write timeout' happen.
    //pub_encoders_.publish(&encoder_msg_);
}

template <typename TMotorController, typename TMotorDriver>
void diffbot::BaseController<TMotorController, TMotorDriver>::write()
{
    // https://www.arduino.cc/reference/en/language/functions/math/map/
    // map(value, fromLow, fromHigh, toLow, toHigh)
    // Map angular wheel joint velocity to motor cmd
    // The /diffbot/mobile_base_controller/linear/x/max_velocity from the ROS parameter server 
    // is convert to max rotational velocity (see init()) and used to map the angular velocity to PWM signals for the motor.
    // Note this is currently unused because the PID helps to keep the velocity to the commanded one.
    //motor_cmd_left_ = map(wheel_cmd_velocity_left_, -max_angular_velocity_, max_angular_velocity_, PWM_MIN, PWM_MAX);
    //motor_cmd_right_ = map(wheel_cmd_velocity_right_, -max_angular_velocity_, max_angular_velocity_, PWM_MIN, PWM_MAX);

    // Compute PID output
    // The value sent to the motor driver is calculated by the PID based on the error between commanded angular velocity vs measured angular velocity
    // The calculated PID ouput value is capped at -/+ MAX_RPM to prevent the PID from having too much error
    motor_cmd_left_ = motor_pid_left_.compute(wheel_cmd_velocity_left_, joint_state_left_.angular_velocity_);
    motor_cmd_right_ = motor_pid_right_.compute(wheel_cmd_velocity_right_, joint_state_right_.angular_velocity_);

    p_motor_controller_left_->setSpeed(motor_cmd_left_);
    p_motor_controller_right_->setSpeed(motor_cmd_right_);
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
    /*
    String log_msg =
            String("\nRead:\n") +
                String("ticks_left_ \t ticks_right_ \t measured_ang_vel_left \t measured_ang_vel_right\n") +
                String(ticks_left_) + String("\t") + String(ticks_right_) + String("\t") +
                String(joint_state_left_.angular_velocity_) + String("\t") + String(joint_state_right_.angular_velocity_) +
            String("\nWrite:\n") + 
                String("motor_cmd_left_ \t motor_cmd_right_ \t pid_left_error \t pid_right_error\n") +
                String(motor_cmd_left_) + String("\t") + String(motor_cmd_right_) + String("\t") +
                //String("pid_left \t pid_right\n") +
                String(motor_pid_left_.error()) + String("\t") + String(motor_pid_right_.error());
    */

    String log_msg =
            String("\nRead:\n") +
                String("ticks_left_: ") + String(ticks_left_) +
                String("\nticks_right_: ") + String(ticks_right_) +
                String("\nmeasured_ang_vel_left: ") + String(joint_state_left_.angular_velocity_) +
                String("\nmeasured_ang_vel_right: ") + String(joint_state_right_.angular_velocity_) +
                String("\nwheel_cmd_velocity_left_: ") + String(wheel_cmd_velocity_left_) +
                String("\nwheel_cmd_velocity_right_: ") + String(wheel_cmd_velocity_right_) +

            String("\nWrite:\n") +
                String("motor_cmd_left_: ") + String(motor_cmd_left_) +
                String("\nmotor_cmd_right_: ") + String(motor_cmd_right_) +
                String("\npid_left_errors (p, i, d): ") + String(motor_pid_left_.proportional()) + String(" ") + String(motor_pid_left_.integral()) + String(" ") + String(motor_pid_left_.derivative()) +
                String("\npid_right_error (p, i, d): ") + String(motor_pid_right_.proportional()) + String(" ") + String(motor_pid_right_.integral()) + String(" ") + String(motor_pid_right_.derivative());
    nh_.loginfo(log_msg.c_str());
}


#endif // DIFFBOT_BASE_CONTROLLER_H