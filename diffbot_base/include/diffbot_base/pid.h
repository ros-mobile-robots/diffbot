

namespace diffbot_base
{
    class PID
    {
    public:
        PID(double kP, double kI, double kD, double setpoint, double output_min, double output_max);
        PID(double kF, double kP, double kI, double kD, double setpoint, double output_min, double output_max);
        double operator()(const double &measured_value, const double &setpoint, const double &dt);
        void setParameters(double kP, double kI, double kD);
        void setOutputLimits(double output_min, double output_max);

    private:
        double kF_;
        double kP_;
        double kI_;
        double kD_;
        double setpoint_;
        double output_min_;
        double output_max_;

        double proportional_;
        double integral_;
        double derivative_;
        double last_error_;
    };

}