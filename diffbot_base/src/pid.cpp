
#include <diffbot_base/pid.h>

namespace diffbot_base
{

    PID::PID(double kP, double kI, double kD, double output_min, double output_max)
    : control_toolbox::Pid(kP, kI, kD)
    {
        init(0.0, kP, kI, kD, output_min, output_max);
    }

    PID::PID(double kF, double kP, double kI, double kD, double output_min, double output_max)
    : control_toolbox::Pid(kP, kI, kD)
    {
        init(kF, kP, kI, kD, output_min, output_max);
    }

    void PID::init(double kF, double kP, double kI, double kD, double output_min, double output_max)
    {
        kF_ = kF;
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;

        error_ = 0.0;

        output_min_ = output_min;
        output_max_ = output_max;

        // Create node handle for dynamic reconfigure
        // TODO use namespace from hardware interface
        ros::NodeHandle nh("diffbot");
        initDynamicReconfig(nh);

        ROS_INFO_STREAM("Initialize PID: F=" << kF_ << ", P=" << kP_ << ", I=" << kI_ << ", D=" << kD_ << ", out_min=" << output_min_ << ", out_max=" << output_max_);

    }

    double PID::operator()(const double &measured_value, const double &setpoint, const ros::Duration &dt)
    {
        // Compute error terms
        error_ = setpoint - measured_value;
        ROS_DEBUG_STREAM_THROTTLE(1, "Error: " << error_);

        // Use control_toolbox::Pid::computeCommand()
        double output = computeCommand(error_, dt);
        ROS_DEBUG_STREAM_THROTTLE(1, "PID computed command: " << output);
 

        // Compute final output including feed forward term
        output = kF_ * setpoint + output;
        output = clamp(output, output_min_, output_max_);

        return output;
    }


    void PID::setParameters(double kP, double kI, double kD)
    {
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;
        ROS_INFO_STREAM("Update PID parameters: P=" << kP_ << ", kI=" << kI_ << ", kD=" << kD_);
    }

    void PID::setOutputLimits(double output_min, double output_max)
    {
        output_min_ = output_min;
        output_max_ = output_max;
        ROS_INFO_STREAM("Update PID output limits: lower=" << output_min_ << ", upper=" << output_max_);
    }


    double PID::clamp(const double& value, const double& lower_limit, const double& upper_limit)
    {
        if (value > upper_limit)
        {
            ROS_DEBUG_STREAM_THROTTLE(1, "Clamp " << value << " to upper limit " << upper_limit);
            return upper_limit;
        }
        else if (value < lower_limit)
        {
            ROS_DEBUG_STREAM_THROTTLE(1, "Clamp " << value << " to lower limit " << upper_limit);
            return lower_limit;
        }

        return value;
    }

}