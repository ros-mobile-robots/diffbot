/*
 * Author: Franz Pucher
 */

#ifndef DIFFBOT_ENCODER_H
#define DIFFBOT_ENCODER_H

#include <Encoder.h>

#include <ros.h>


namespace diffbot
{
    struct JointState
    {
        double angular_position_;
        double angular_velocity_;
    };


    /** \brief Decorates the Teensy Encoder Library to read the angular wheel velocity
     * from quadrature wheel encoders.
     * 
     * This class is composed of \ref ::Encoder from https://www.pjrc.com/teensy/td_libs_Encoder.html,
     * which is capable of reading rising and falling edges of two Hall effect signals. This yields
     * the highest possible tick count (encoder resolution) from the encoders.
     * Take the DG01D-E motor/encoder, that has a 6 pole magnetic disk (observed with magnetic paper) and therefore 3 pulses per revolution (ppr).
     * Reading both edges (rising and falling) of both channels in the code, we can observe roughly 542 ticks at the wheel output shaft.
     * Because the DG01D-E motor/encoder has a 48:1 gear ratio (according to its datasheet) we round the tick count up to 576 counts per wheel revolution). 
     * So 576 counts / 48:1 gear ratio / 2 channels / 2 edges = 3 pulses per revolution at the motor shaft. 
     * Having an encoder with more ppr would increase the encoder resolution. For example let's say we we have an encoder with 7 ppr, 
     * then we get the following output resolution at the wheel: 7 ppr (at motor shaft) * 48:1 gear ratio * 2 channels * 2 edges = 1344 ppr (measured at wheel).
     */
    class Encoder
    {
    public:
        // Teensy Encoder class that is capable of reading rising and falling edges of two Hall effect signals.
        ::Encoder encoder;

        /** \brief Construct a diffbot::Encoder providing access to quadrature encoder ticks and angular joint velocity.
         * 
         * \param nh reference to the main ros::NodeHandle to compute the velocity from time and ticks or angle (s = v * t)
         * \param pin1 Pin of the first Hall effect sensor
         * \param pin2 Pin of the second Hall effect sensor
         * \param encoder_resolution number of tick counts for one full revolution of the wheel (not the motor shaft). Keep track of gear reduction ratio.
         */
        Encoder(ros::NodeHandle& nh, uint8_t pin1, uint8_t pin2, int encoder_resolution);

        /** \brief get revolutions per minute
         *
         * Calculates the wheels revolution per minute using the encoder ticks.
         * 
         * Based on https://github.com/linorobot/linorobot/blob/53bc4abf150632fbc6c28cdfcc207caa0f81d2b1/teensy/firmware/lib/encoder/Encoder.h
         * 
         * \returns revolutions per minute
         */
        int getRPM();


        double angularPosition();

        /** \brief Get the measure the angular joint velocity
         *
         * Calculates the angular velocity of the wheel joint using the encoder ticks.
         * 
         * \returns angular wheel joint velocity (rad/s)
         */
        double angularVelocity();


        JointState jointState();

        /** \brief Convert number of encoder ticks to angle in radians 
         *
         * Calculates the current tick count of the encoder to its absolute angle in radians
         * using the \ref encoder_resolution_.
         * 
         * \param ticks tick count from an encoder which is converted to a corresponding absolute angle.
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
         * Mainly used to reset the encoder back to zero.
         * 
         * \param p encoder ticks
         */
        inline void write(int32_t p) { encoder.write(p); };

        /** \brief Setter for encoder resolution
         * 
         * Used to initialize the encoder with a new resolution, e.g. obtained
         * from the ROS parameter server.
         * 
         * \param resolution value to which the encoder tick count shoudl be set
         */
        inline void resolution(int resolution) { encoder_resolution_ = resolution; };

        /** \brief Getter for encoder resolution
         * 
         * Returns the currently set encder resolution.
         */
        inline int resolution() { return encoder_resolution_; };


        JointState joint_state_;

    private:
        // ROS node handle, which provides the current time to compute the angular velocity from the current tick count
        ros::NodeHandle& nh_;
        // Number of tick counts for one full revolution of the wheel (not the motor shaft). Keep track of gear reduction ratio.
        int encoder_resolution_;
        // Previous time when the \ref getRPM or \ref angularVelocity method was called to calculated the delta update time.
        ros::Time prev_update_time_;
        // Previous encoder tick count when the \ref getRPM or \ref angularVelocity method was called to calculated the delta tick count.
        long prev_encoder_ticks_;
    };
}

#endif // DIFFBOT_ENCODER_H