#include <diffbot_base/diffbot_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

//#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

#include <iomanip>
 
namespace diffbot_base
{
    DiffBotHWInterface::DiffBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : name_("hardware_interface")
        , nh_(nh)
    { 
        // Initialization of the robot's resources (joints, sensors, actuators) and
        // interfaces can be done here or inside init().
        // E.g. parse the URDF for joint names & interfaces, then initialize them
        // Check if the URDF model needs to be loaded
        if (urdf_model == NULL)
            loadURDF(nh, "robot_description");
        else
            urdf_model_ = urdf_model;

        // Load rosparams
        ros::NodeHandle rpnh(nh_, name_);
        std::size_t error = 0;
        // Code API of rosparam_shortcuts:
        // http://docs.ros.org/en/noetic/api/rosparam_shortcuts/html/namespacerosparam__shortcuts.html#aa6536fe0130903960b1de4872df68d5d
        error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/wheel_radius", wheel_radius_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/linear/x/max_velocity", max_velocity_);
        // Get additional parameters from the diffbot_base/config/base.yaml which is stored on the parameter server
        error += !rosparam_shortcuts::get(name_, nh_, "encoder_resolution", encoder_resolution_);
        error += !rosparam_shortcuts::get(name_, nh_, "gain", gain_);
        error += !rosparam_shortcuts::get(name_, nh_, "trim", trim_);
        error += !rosparam_shortcuts::get(name_, nh_, "motor_constant", motor_constant_);
        error += !rosparam_shortcuts::get(name_, nh_, "pwm_limit", pwm_limit_);
        error += !rosparam_shortcuts::get(name_, nh_, "debug/hardware_interface", debug_);
        rosparam_shortcuts::shutdownIfError(name_, error);

        wheel_diameter_ = 2.0 * wheel_radius_;
        //max_velocity_ = 0.2; // m/s
        // ros_control RobotHW needs velocity in rad/s but in the config its given in m/s
        max_velocity_ = linearToAngular(max_velocity_);


        ROS_INFO_STREAM("mobile_base_controller/wheel_radius: " << wheel_radius_);
        ROS_INFO_STREAM("mobile_base_controller/linear/x/max_velocity: " << max_velocity_);
        ROS_INFO_STREAM("encoder_resolution: " << encoder_resolution_);
        ROS_INFO_STREAM("gain: " << gain_);
        ROS_INFO_STREAM("trim: " << trim_);
        ROS_INFO_STREAM("motor_constant: " << motor_constant_);
        ROS_INFO_STREAM("pwm_limit: " << pwm_limit_);

        // Setup publisher for the motor driver 
        pub_left_motor_value_ = nh_.advertise<std_msgs::Int32>("motor_left", 10);
        pub_right_motor_value_ = nh_.advertise<std_msgs::Int32>("motor_right", 10);

        //Setup publisher for angular wheel joint velocity commands
        pub_wheel_cmd_velocities_ = nh_.advertise<diffbot_msgs::WheelsCmdStamped>("wheel_cmd_velocities", 10);

        // Setup publisher for the IMU format transform
        imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/processed", 50);
        
        // Setup publisher to reset wheel encoders (used during first launch of the hardware interface)
        pub_reset_encoders_ = nh_.advertise<std_msgs::Empty>("reset", 10);
        // Setup subscriber for the wheel encoders
        sub_encoder_ticks_ = nh_.subscribe("encoder_ticks", 10, &DiffBotHWInterface::encoderTicksCallback, this);
        sub_measured_joint_states_ = nh_.subscribe("measured_joint_states", 10, &DiffBotHWInterface::measuredJointStatesCallback, this);

        //setup subscriber for the IMU
        sub_imu_data_ = nh_.subscribe("imu", 10, &DiffBotHWInterface::IMUDataCallback, this);

        // Initialize the hardware interface
        init(nh_, nh_);

        // Wait for encoder messages being published
        isReceivingMeasuredJointStates(ros::Duration(10));
    }

 
    bool DiffBotHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        ROS_INFO("Initializing DiffBot Hardware Interface ...");
        num_joints_ = joint_names_.size();
        ROS_INFO("Number of joints: %d", (int)num_joints_);
        std::array<std::string, NUM_JOINTS> motor_names = {"left_motor", "right_motor"};
        for (unsigned int i = 0; i < num_joints_; i++)
        {
            // Create a JointStateHandle for each joint and register them with the 
            // JointStateInterface.
            hardware_interface::JointStateHandle joint_state_handle(joint_names_[i],
                                                                    &joint_positions_[i], 
                                                                    &joint_velocities_[i],
                                                                    &joint_efforts_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            // Create a JointHandle (read and write) for each controllable joint
            // using the read-only joint handles within the JointStateInterface and 
            // register them with the JointVelocityInterface.
            hardware_interface::JointHandle joint_handle(joint_state_handle, &joint_velocity_commands_[i]);
            velocity_joint_interface_.registerHandle(joint_handle);

            // Initialize joint states with zero values
            joint_positions_[i] = 0.0;
            joint_velocities_[i] = 0.0;
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller

            joint_velocity_commands_[i] = 0.0;

            // Initialize encoder_ticks_ & IMU to zero because receiving meaningful
            // tick values from the microcontroller might take some time
            encoder_ticks_[i] = 0.0;
            IMU_data_[i] = 0.0;
            measured_joint_states_[i].angular_position_ = 0.0;
            measured_joint_states_[i].angular_velocity_ = 0.0;

            // Initialize the pid controllers for the motors using the robot namespace
            std::string pid_namespace = "pid/" + motor_names[i];
            ROS_INFO_STREAM("pid namespace: " << pid_namespace);
            ros::NodeHandle nh(root_nh, pid_namespace);
            // TODO implement builder pattern to initialize values otherwise it is hard to see which parameter is what.
            pids_[i].init(nh, 0.8, 0.35, 0.5, 0.01, 3.5, -3.5, false, max_velocity_, -max_velocity_);
            pids_[i].setOutputLimits(-max_velocity_, max_velocity_);
        }

        // Register the JointStateInterface containing the read only joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&joint_state_interface_);

        // Register the JointVelocityInterface containing the read/write joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&velocity_joint_interface_);

        ROS_INFO("... Done Initializing DiffBot Hardware Interface");

        return true;
    }

    void DiffBotHWInterface::read(const ros::Time& time, const ros::Duration& period)
    {
        //ROS_INFO_THROTTLE(1, "Read");
        ros::Duration elapsed_time = period;

        // Read from robot hw (motor encoders)
        // Fill joint_state_* members with read values
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            joint_positions_[i] = measured_joint_states_[i].angular_position_;
            joint_velocities_[i] = measured_joint_states_[i].angular_velocity_;
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller
        }

        if (debug_)
        {
            const int width = 10;
            const char sep = ' ';
            std::stringstream ss;
            ss << std::left << std::setw(width) << std::setfill(sep) << "Read" << std::left << std::setw(width) << std::setfill(sep) << "ticks" << std::left << std::setw(width) << std::setfill(sep) << "angle" << std::left << std::setw(width) << std::setfill(sep) << "velocity" << std::endl;
            ss << std::left << std::setw(width) << std::setfill(sep) << "j0:" << std::left << std::setw(width) << std::setfill(sep) << encoder_ticks_[0] << std::left << std::setw(width) << std::setfill(sep) << joint_positions_[0] << std::left << std::setw(width) << std::setfill(sep) << joint_velocities_[0] << std::endl;
            ss << std::left << std::setw(width) << std::setfill(sep) << "j1:" << std::left << std::setw(width) << std::setfill(sep) << encoder_ticks_[1] << std::left << std::setw(width) << std::setfill(sep) << joint_positions_[1] << std::left << std::setw(width) << std::setfill(sep) << std::setfill(sep) << joint_velocities_[1];
            ROS_INFO_STREAM(std::endl << ss.str());
            //printState();
        }

        //  Publish IMU values to handle information to EKF Filter
        PublishIMUtoFilter();
    }

    void DiffBotHWInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;
        // Write to robot hw
        // joint velocity commands from ros_control's RobotHW are in rad/s

        // adjusting k by gain and trim
        double motor_constant_right_inv = (gain_ + trim_) / motor_constant_;
        double motor_constant_left_inv = (gain_ - trim_) / motor_constant_;


        joint_velocity_commands_[0] = joint_velocity_commands_[0] * motor_constant_left_inv;
        joint_velocity_commands_[1] = joint_velocity_commands_[1] * motor_constant_right_inv;


        // Publish the desired (commanded) angular wheel joint velocities
        diffbot_msgs::WheelsCmdStamped wheel_cmd_msg;
        wheel_cmd_msg.header.stamp = ros::Time::now();
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            wheel_cmd_msg.wheels_cmd.angular_velocities.joint.push_back(joint_velocity_commands_[i]);
        }

        pub_wheel_cmd_velocities_.publish(wheel_cmd_msg);

        // The following code provides another velocity commands interface
        // With it a motor driver node can directly subscribe to the desired velocities.

        // Convert the velocity command to a percentage value for the motor
        // This maps the velocity to a percentage value which is used to apply
        // a percentage of the highest possible battery voltage to each motor.
        std_msgs::Int32 left_motor;
        std_msgs::Int32 right_motor;

        double pid_outputs[NUM_JOINTS];
        double motor_cmds[NUM_JOINTS] ;
        pid_outputs[0] = pids_[0](joint_velocities_[0], joint_velocity_commands_[0], period);
        pid_outputs[1] = pids_[1](joint_velocities_[1], joint_velocity_commands_[1], period);

        motor_cmds[0] = pid_outputs[0] / max_velocity_ * 100.0;
        motor_cmds[1] = pid_outputs[1] / max_velocity_ * 100.0;
        left_motor.data = motor_cmds[0];
        right_motor.data = motor_cmds[1];

        // Calibrate motor commands to deal with different gear friction in the
        // left and right motors and possible differences in the wheels.
        // Add calibration offsets to motor output in low regions
        // To tune these offset values command the robot to drive in a straight line and
        // adjust if it isn't going straight.
        // int left_offset = 10;
        // int right_offset = 5;
        // int threshold = 55;
        // if (0 < left_motor.data && left_motor.data < threshold)
        // {
        //     // the second part of the multiplication lets the offset decrease with growing motor values
        //     left_motor.data += left_offset * (threshold - left_motor.data) / threshold;
        // }
        // if (0 < right_motor.data && right_motor.data < threshold)
        // {
        //     // the second part of the multiplication lets the offset decrease with growing motor values
        //     right_motor.data += right_offset * (threshold - right_motor.data) / threshold;
        // }

        pub_left_motor_value_.publish(left_motor);
        pub_right_motor_value_.publish(right_motor);


        if (debug_)
        {
            const int width = 10;
            const char sep = ' ';
            std::stringstream ss;
            // Header
            ss << std::left << std::setw(width) << std::setfill(sep) << "Write"
            << std::left << std::setw(width) << std::setfill(sep) << "velocity"
            << std::left << std::setw(width) << std::setfill(sep) << "p_error"
            << std::left << std::setw(width) << std::setfill(sep) << "i_error"
            << std::left << std::setw(width) << std::setfill(sep) << "d_error"
            << std::left << std::setw(width) << std::setfill(sep) << "pid out"
            << std::left << std::setw(width) << std::setfill(sep) << "percent"
            << std::endl;
            double p_error, i_error, d_error;
            for (int i = 0; i < NUM_JOINTS; ++i)
            {
                pids_[i].getCurrentPIDErrors(&p_error, &i_error, &d_error);

                // Joint i
                std::string j = "j" + std::to_string(i) + ":";
                ss << std::left << std::setw(width) << std::setfill(sep) << j
                << std::left << std::setw(width) << std::setfill(sep) << joint_velocity_commands_[i]
                << std::left << std::setw(width) << std::setfill(sep) << p_error
                << std::left << std::setw(width) << std::setfill(sep) << i_error
                << std::left << std::setw(width) << std::setfill(sep) << d_error
                << std::left << std::setw(width) << std::setfill(sep) << pid_outputs[i]
                << std::left << std::setw(width) << std::setfill(sep) << motor_cmds[i]
                << std::endl;
            }
            ROS_INFO_STREAM(std::endl << ss.str());
        }
    }

    bool DiffBotHWInterface::isReceivingMeasuredJointStates(const ros::Duration &timeout)
    {
        ROS_INFO("Get number of measured joint states publishers");

        ros::Time start = ros::Time::now();
        int num_publishers = sub_measured_joint_states_.getNumPublishers();
        ROS_INFO("Waiting for measured joint states being published...");
        while ((num_publishers == 0) && (ros::Time::now() < start + timeout))
        {
            ros::Duration(0.1).sleep();
            num_publishers = sub_measured_joint_states_.getNumPublishers();
        }
        if (num_publishers == 0)
        {
            ROS_ERROR("No measured joint states publishers. Timeout reached.");
        }
        else
        {
            ROS_INFO_STREAM("Number of measured joint states publishers: " << num_publishers);
        }

        ROS_INFO("Publish /reset to encoders");
        std_msgs::Empty msg;
        pub_reset_encoders_.publish(msg);

        return (num_publishers > 0);
    }

    void DiffBotHWInterface::loadURDF(const ros::NodeHandle &nh, std::string param_name)
    {
        std::string urdf_string;
        urdf_model_ = new urdf::Model();

        // search and wait for robot_description on param server
        while (urdf_string.empty() && ros::ok())
        {
            std::string search_param_name;
            if (nh.searchParam(param_name, search_param_name))
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << search_param_name);
                nh.getParam(search_param_name, urdf_string);
            }
            else
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << param_name);
                nh.getParam(param_name, urdf_string);
            }

            usleep(100000);
        }

        if (!urdf_model_->initString(urdf_string))
            ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
        else
            ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
    }

    void DiffBotHWInterface::printState()
    {
        // WARNING: THIS IS NOT REALTIME SAFE
        // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
        ROS_INFO_STREAM_THROTTLE(1, std::endl << printStateHelper());
    }

    std::string DiffBotHWInterface::printStateHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);

        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << "j" << i << ": " << std::fixed << joint_positions_[i] << "\t ";
            ss << std::fixed << joint_velocities_[i] << "\t ";
            ss << std::fixed << joint_efforts_[i] << std::endl;
        }
        return ss.str();
    }

    std::string DiffBotHWInterface::printCommandHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);
        ss << "    position     velocity         effort  \n";
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << std::fixed << joint_velocity_commands_[i] << "\t ";
        }
        return ss.str();
    }


    /// Process updates from encoders
    void DiffBotHWInterface::encoderTicksCallback(const diffbot_msgs::EncodersStamped::ConstPtr& msg_encoder)
    {
        /// Update current encoder ticks in encoders array
        encoder_ticks_[0] = msg_encoder->encoders.ticks[0];
        encoder_ticks_[1] = msg_encoder->encoders.ticks[1];
        ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << encoder_ticks_[0]);
        ROS_DEBUG_STREAM_THROTTLE(1, "Right encoder ticks: " << encoder_ticks_[1]);
    }

    /// Process updates from imu
    void DiffBotHWInterface::IMUDataCallback(const diffbot_msgs::IMU::ConstPtr& msg_imu)
    {
        /// update current data from IMU
        orientation[0] = msg_imu->imu_msg[0];
        orientation[1] = msg_imu->imu_msg[1];
        orientation[2] = msg_imu->imu_msg[2];
        orientation[3] = msg_imu->imu_msg[8];
        
        angular_velocity[0] = msg_imu->imu_msg[6];
        angular_velocity[1] = msg_imu->imu_msg[7];
        angular_velocity[2] = msg_imu->imu_msg[8];

        linear_acceleration[0] = msg_imu->imu_msg[3];
        linear_acceleration[1] = msg_imu->imu_msg[4];
        linear_acceleration[2] = msg_imu->imu_msg[5];

        ROS_DEBUG_STREAM_THROTTLE(1, "x orientation: " << encoder_ticks_[0]);
        ROS_DEBUG_STREAM_THROTTLE(1, "y orientation: " << encoder_ticks_[1]);
        ROS_DEBUG_STREAM_THROTTLE(1, "z orientation: " << encoder_ticks_[2]);
        ROS_DEBUG_STREAM_THROTTLE(1, "w orientation: " << encoder_ticks_[3]);
        ROS_DEBUG_STREAM_THROTTLE(1, "x angular velocity: " << encoder_ticks_[0]);
        ROS_DEBUG_STREAM_THROTTLE(1, "y angular velocity: " << encoder_ticks_[1]);
        ROS_DEBUG_STREAM_THROTTLE(1, "z angular velocity: " << encoder_ticks_[2]);
        ROS_DEBUG_STREAM_THROTTLE(1, "x linear accelaration: " << encoder_ticks_[0]);
        ROS_DEBUG_STREAM_THROTTLE(1, "y linear accelaration: " << encoder_ticks_[1]);
        ROS_DEBUG_STREAM_THROTTLE(1, "z linear accelaration: " << encoder_ticks_[2]);
    }
    

    void DiffBotHWInterface::PublishIMUtoFilter()
    {
        imu_pub_.publish(imu);
        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = "imu";

        imu.orientation.x = orientation[0];
        imu.orientation.y = orientation[1];
        imu.orientation.z = orientation[2];
        imu.orientation.w = orientation[3];

        imu.angular_velocity.x = angular_velocity[0];
        imu.angular_velocity.y = angular_velocity[1];
        imu.angular_velocity.z = angular_velocity[2];

        imu.linear_acceleration.x = linear_acceleration[0];
        imu.linear_acceleration.y = linear_acceleration[1];
        imu.linear_acceleration.z = linear_acceleration[2];
    }
    void DiffBotHWInterface::measuredJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg_joint_states)
    {
        /// Update current encoder ticks in encoders array
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            measured_joint_states_[i].angular_position_ = msg_joint_states->position[i];
            measured_joint_states_[i].angular_velocity_ = msg_joint_states->velocity[i];
        }
        //ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << encoder_ticks_[0]);
        //ROS_DEBUG_STREAM_THROTTLE(1, "Right encoder ticks: " << encoder_ticks_[1]);
    }


    double DiffBotHWInterface::ticksToAngle(const int &ticks) const
    {
        // Convert number of encoder ticks to angle in radians
        double angle = (double)ticks * (2.0*M_PI / encoder_resolution_);
        ROS_DEBUG_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
	    return angle;
    }

    double DiffBotHWInterface::normalizeAngle(double &angle) const
    {
        // https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
        angle = fmod(angle, 2.0*M_PI);

        if (angle < 0)
            angle += 2.0*M_PI;

        ROS_DEBUG_STREAM_THROTTLE(1, "Normalized angle: " << angle);
        return angle;
    }


    double DiffBotHWInterface::linearToAngular(const double &distance) const
    {
        return distance / wheel_diameter_ * 2.0;
    }

    double DiffBotHWInterface::angularToLinear(const double &angle) const
    {
        return angle * wheel_diameter_ / 2.0;
    }

};
