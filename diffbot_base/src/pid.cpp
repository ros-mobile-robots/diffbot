
#include <diffbot_base/pid.h>

namespace diffbot_base
{

    PID::PID(double kP, double kI, double kD, double setpoint, double output_min, double output_max)
    {
        kF_ = 0.0;
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;

        setpoint_ = setpoint;

        output_min_ = output_min;
        output_max_ = output_max;

        proportional_ = 0.0;
        integral_ = 0.0;
        derivative_ = 0.0;
    }

    PID::PID(double kF, double kP, double kI, double kD, double setpoint, double output_min, double output_max)
    {
        kF_ = kF;
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;

        setpoint_ = setpoint;

        output_min_ = output_min;
        output_max_ = output_max;

        proportional_ = 0.0;
        integral_ = 0.0;
        derivative_ = 0.0;
    }

    double PID::operator()(const double &measured_value, const double &setpoint, const double &dt)
    {
        if (dt <= 0)
        {
            throw std::domain_error("dt must be greater than zero.");
        }

        setpoint_ = setpoint;
        process_value_ = measured_value;

        // Compute error terms
        double error = setpoint_ - process_value_;
        double delta_error = error - last_error_;

        // Compute the proportional term
        proportional_ = error;

        // Compute integral term
        integral_ += error * dt;
        // TODO: avoid integral windup
        //integral_ = clamp(integral_, output_min_, output_max_)  

        // Compute derivative term
        derivative_ = delta_error / dt;

        // Compute final output including feed forward term
        output = kF * setpoint + kP_ * proportional_ + kI_ * integral_ + kD_ * derivative_;
        //output = clamp(output, output_min_, output_max_)

        // Keep track of state
        last_error_ = error;

        return output
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


    double PID::clamp(value, lower_limit, upper_limit)
    {
        if (value > upper_limit)
        {
            return upper_limit;
        }
        else if (value < lower_limit)
        {
            return lower
        }

        return value
    }

}