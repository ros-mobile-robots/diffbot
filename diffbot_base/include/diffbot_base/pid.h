#include <ros/duration.h>
#include <control_toolbox/pid.h>

namespace diffbot_base
{
    class PID : public control_toolbox::Pid
    {
    public:
        PID(double kP, double kI, double kD, double output_min, double output_max);
        PID(double kF, double kP, double kI, double kD, double output_min, double output_max);
        double operator()(const double &measured_value, const double &setpoint, const ros::Duration &dt);
        void setParameters(double kP, double kI, double kD);
        void setOutputLimits(double output_min, double output_max);
        double clamp(const double& value, const double& lower_limit, const double& upper_limit);

    private:
        double kF_;
        double kP_;
        double kI_;
        double kD_;
        double output_min_;
        double output_max_;
    };

}