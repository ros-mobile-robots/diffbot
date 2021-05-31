/*
 * Author: Franz Pucher
 */

#ifndef DIFFBOT_ENCODER_H
#define DIFFBOT_ENCODER_H

#include <Encoder.h>


namespace diffbot 
{
    class Encoder
    {
    public:
        ::Encoder encoder;

        Encoder(uint8_t pin1, uint8_t pin2, int encoder_resolution);

        /** \brief get revolutions per minute
         *
         * Calculates the wheels revolution per minute using the encoder ticks.
         * 
         * Based on https://github.com/linorobot/linorobot/blob/53bc4abf150632fbc6c28cdfcc207caa0f81d2b1/teensy/firmware/lib/encoder/Encoder.h
         * 
         * \returns revolutions per minute
         */
        int getRPM();

        /** \brief Get the measure the angular joint velocity
         *
         * Calculates the angular velocity of the wheel joint using the encoder ticks.
         * 
         * \returns angular wheel joint velocity (rad/s)
         */
        float angularVelocity();

        /** \brief Convert number of encoder ticks to angle in radians 
         *
         * Calculates the current tick count of the encoder to its absolute angle in radians.
         * 
         * \returns angle corresponding to encoder ticks (rad)
         */
        double ticksToAngle(const int &ticks) const;

        /** \brief Read the current encoder tick count
         * 
         * \returns encoder ticks
         */
        inline int32_t read() { return encoder.read(); };

        /** \brief Set the encoder tick count
         * 
         * \param p encoder ticks
         */
        inline void write(int32_t p) { encoder.write(p); };

    private:
        int encoder_resolution_;
        unsigned long prev_update_time_;
        long prev_encoder_ticks_;
    };
}

#endif // DIFFBOT_ENCODER_H