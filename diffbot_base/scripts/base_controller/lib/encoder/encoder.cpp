#include "encoder.h"


diffbot::Encoder::Encoder(ros::NodeHandle& nh, uint8_t pin1, uint8_t pin2, int encoder_resolution)
  : nh_(nh)
  , encoder(pin1, pin2)
  , encoder_resolution_(encoder_resolution)
  , prev_update_time_(0, 0)
  , prev_encoder_ticks_(0)
{

}


float diffbot::Encoder::angularVelocity()
{
    long encoder_ticks = encoder.read();
    // This function calculates the motor's rotational (angular) velocity based on encoder ticks and delta time
    ros::Time current_time = nh_.now();
    ros::Duration dt = current_time - prev_update_time_;

    // Convert the delta time to seconds
    double dts = dt.toSec();

    //calculate wheel's speed (RPM)
    double delta_ticks = encoder_ticks - prev_encoder_ticks_;
    double delta_angle = ticksToAngle(delta_ticks);
    double angular_velocity = delta_angle / dts;

    prev_update_time_ = current_time;
    prev_encoder_ticks_ = encoder_ticks;

    return angular_velocity;
}

double diffbot::Encoder::ticksToAngle(const int &ticks) const
{
  // Convert number of encoder ticks to angle in radians
  double angle = (double)ticks * (2.0*M_PI / encoder_resolution_);
  return angle;
}


int diffbot::Encoder::getRPM()
{
    long encoder_ticks = encoder.read();
    //this function calculates the motor's RPM based on encoder ticks and delta time
    ros::Time current_time = nh_.now();
    ros::Duration dt = current_time - prev_update_time_;

    //convert the time from milliseconds to minutes
    double dtm = dt.toSec() / 60;
    double delta_ticks = encoder_ticks - prev_encoder_ticks_;

    //calculate wheel's speed (RPM)

    prev_update_time_ = current_time;
    prev_encoder_ticks_ = encoder_ticks;

    return (delta_ticks / encoder_resolution_) / dtm;
}