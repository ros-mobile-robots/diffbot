
#include <diffbot_base/pid.h>

namespace diffbot_base
{

    PID::PID(double kP, double kI, double kD, double output_min, double output_max)
    : control_toolbox::Pid(kP, kI, kD)
    {
        kF_ = 0.0;
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;

        output_min_ = output_min;
        output_max_ = output_max;
    }

    PID::PID(double kF, double kP, double kI, double kD, double output_min, double output_max)
    : control_toolbox::Pid(kP, kI, kD)
    {
        kF_ = kF;
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;

        output_min_ = output_min;
        output_max_ = output_max;
    }

    double PID::operator()(const double &measured_value, const double &setpoint, const ros::Duration &dt)
    {
        // Compute error terms
        double error = setpoint - measured_value;

        // Use control_toolbox::Pid::computeCommand()
        double output = computeCommand(error, dt);
 

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
    }

    void PID::setOutputLimits(double output_min, double output_max)
    {
        output_min_ = output_min;
        output_max_ = output_max;
    }


    double PID::clamp(const double& value, const double& lower_limit, const double& upper_limit)
    {
        if (value > upper_limit)
        {
            return upper_limit;
        }
        else if (value < lower_limit)
        {
            return lower_limit;
        }

        return value;
    }

}