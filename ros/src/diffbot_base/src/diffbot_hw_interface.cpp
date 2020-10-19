#include <diffbot_base/diffbot_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

//#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
 
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
        rosparam_shortcuts::shutdownIfError(name_, error);

        wheel_diameter_ = 2.0 * wheel_radius_;
        //max_velocity_ = 0.2; // m/s
        // ros_control RobotHW needs velocity in rad/s but in the config its given in m/s
        max_velocity_ = linearToAngular(max_velocity_);

        // Setup publisher for the motor driver 
        pub_left_motor_value_ = nh_.advertise<std_msgs::Int32>("motor_left", 1);
        pub_right_motor_value_ = nh_.advertise<std_msgs::Int32>("motor_right", 1);

        // Setup subscriber for the wheel encoders
        sub_left_encoder_ticks_ = nh_.subscribe("ticks_left", 1, &DiffBotHWInterface::leftEncoderTicksCallback, this);
        sub_right_encoder_ticks_ = nh_.subscribe("ticks_right", 1, &DiffBotHWInterface::rightEncoderTicksCallback, this);

        // Initialize the hardware interface
        init(nh_, nh_);
    }

 
    bool DiffBotHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        ROS_INFO("Initializing DiffBot Hardware Interface ...");
        num_joints_ = joint_names_.size();
        ROS_INFO("Number of joints: %d", (int)num_joints_);
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

        // TODO read from robot hw

        // TODO fill joint_state_* members with read values
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            double wheel_angle = ticksToAngle(encoder_ticks_[0]);
            double wheel_angle_delta = normalizeAngle(wheel_angle);
            
            joint_positions_[i] += wheel_angle_delta;
            joint_velocities_[i] = wheel_angle_delta / period.toSec();
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller
        }

        printState();
    }

    void DiffBotHWInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;
        // TODO write to robot hw
        // joint velocity commands from ros_control's RobotHW are in rad/s
        // Convert the velocity command to a percentage value for the motor
        std_msgs::Int32 left_motor;
        std_msgs::Int32 right_motor;
        left_motor.data = joint_velocity_commands_[0] / max_velocity_ * 100.0;
        right_motor.data = joint_velocity_commands_[1] / max_velocity_ * 100.0;

        ROS_INFO_STREAM_THROTTLE(1, "Write: left motor: " << left_motor.data << ", right motor: " << right_motor.data);
        pub_left_motor_value_.publish(left_motor);
        pub_right_motor_value_.publish(right_motor);
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
    void DiffBotHWInterface::leftEncoderTicksCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        encoder_ticks_[0] = msg->data;
        ROS_INFO_STREAM_THROTTLE(1, "Left encoder ticks: " << msg->data);
    }

    void DiffBotHWInterface::rightEncoderTicksCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        encoder_ticks_[1] = msg->data;
        ROS_INFO_STREAM_THROTTLE(1, "Right encoder ticks: " << msg->data);
    }


    double DiffBotHWInterface::ticksToAngle(const int &ticks) const
    {
        // Convert number of encoder ticks to angle in radians
        double angle = (double)ticks * (2.0*M_PI / 542.0);
        ROS_INFO_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
	    return angle;
    }

    double DiffBotHWInterface::normalizeAngle(double &angle) const
    {
        // https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
        angle = fmod(angle, 2.0*M_PI);

        if (angle < 0)
            angle += 2.0*M_PI;

        ROS_INFO_STREAM_THROTTLE(1, "Normalized Angle" << angle);
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