
#include <diffbot_base/pid.h>

namespace diffbot_base
{
    PID::PID(double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max, double out_min)
    : control_toolbox::Pid()
    {
        f_ = 0.0;
        initPid(p, i, d, i_max, i_min, antiwindup);
        error_ = 0.0;

        out_min_ = out_min;
        out_max_ = out_max;
    }

    void PID::init(ros::NodeHandle& nh, double f, double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max, double out_min)
    {
        ROS_INFO("Initialize PID");
        f_ = f;
        initPid(p, i, d, i_max, i_min, antiwindup);
        error_ = 0.0;

        out_min_ = out_min;
        out_max_ = out_max;

        initDynamicReconfig(nh);

        Gains gains = getGains();
        ROS_INFO_STREAM("Initialized PID: F=" << f << ", P=" << gains.p_gain_ << ", I=" << gains.i_gain_ << ", D=" << gains.d_gain_ << ", out_min=" << out_min_ << ", out_max=" << out_max_);

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
        output = f_ * setpoint + output;
        output = clamp(output, out_min_, out_max_);

        return output;
    }


    void PID::setGains(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup)
    {
        f_ = f;
        setGains(p, i, d, i_max, i_min, antiwindup);

        Gains gains = getGains();
        ROS_INFO_STREAM("Update PID Gains: F=" << f << ", P=" << gains.p_gain_ << ", I=" << gains.i_gain_ << ", D=" << gains.d_gain_ << ", out_min=" << out_min_ << ", out_max=" << out_max_);

    }

    void PID::setOutputLimits(double output_min, double output_max)
    {
        out_min_ = output_min;
        out_max_ = output_max;
        ROS_INFO_STREAM("Update PID output limits: lower=" << out_min_ << ", upper=" << out_max_);
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