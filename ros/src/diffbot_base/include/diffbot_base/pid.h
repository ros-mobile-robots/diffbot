

namespace diffbot_base
{
    class PID
    {
    public:
        PID(double kp, double ki, double kd, double setpoint, double output_min, double output_max);
        double operator()(double measured_value, double setpoint);
        void setParameters(double kp, double ki, double kd);

    private:
        double kp_;
        double ki_;
        double kd_;
        double min_val_;
        double max_val_;

        double integral_;
        double derivative_;
        double prev_error_;
    };

}