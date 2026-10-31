#include "pid.h"

diffbot::PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd),
    proportional_(0),
    integral_(0),
    derivative_(0),
    prev_error_(0)
{
}

double diffbot::PID::compute(float setpoint, float measured_value)
{
    double error;
    double pid;

    //setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    proportional_ = error;
    integral_ += error;
    derivative_ = error - prev_error_;

    if(setpoint == 0 && error == 0)
    {
        integral_ = 0;
    }

    pid = (kp_ * proportional_) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    // Limit the output to the motor driver's range
    if (pid < min_val_)
    {
        return min_val_;
    }
    if (pid > max_val_)
    {
        return max_val_;
    }
    return pid;
}

void diffbot::PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void diffbot::PID::reset()
{
    proportional_ = 0;
    integral_ = 0;
    derivative_ = 0;
    prev_error_ = 0;
}
