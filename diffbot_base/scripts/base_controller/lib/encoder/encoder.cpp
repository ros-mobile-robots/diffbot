#include "encoder.h"


diffbot::Encoder::Encoder(uint8_t pin1, uint8_t pin2)
  : encoder(pin1, pin2)
{

}


int diffbot::Encoder::getRPM()
{
    long encoder_ticks = encoder.read();
    //this function calculates the motor's RPM based on encoder ticks and delta time
    unsigned long current_time = millis();
    unsigned long dt = current_time - prev_update_time_;

    //convert the time from milliseconds to minutes
    double dtm = (double)dt / 60000;
    double delta_ticks = encoder_ticks - prev_encoder_ticks_;

    //calculate wheel's speed (RPM)

    prev_update_time_ = current_time;
    prev_encoder_ticks_ = encoder_ticks;

    return (delta_ticks / counts_per_rev_) / dtm;
}