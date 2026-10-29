#ifndef PID_H
#define PID_H

namespace diffbot {

    class PID
    {
        public:
            PID(float min_val, float max_val, float kp, float ki, float kd);
            double compute(float setpoint, float measured_value);
            void updateConstants(float kp, float ki, float kd);

            inline double proportional() { return proportional_; };
            inline double integral() { return integral_; };
            inline double derivative() { return derivative_; };
            inline double prev_error() { return prev_error_; };

        private:
            float min_val_;
            float max_val_;
            float kp_;
            float ki_;
            float kd_;
            double proportional_;
            double integral_;
            double derivative_;
            double prev_error_;
    };
}

#endif