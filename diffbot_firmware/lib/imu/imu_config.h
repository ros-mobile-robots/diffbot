#ifndef DIFFBOT_IMU_CONFIG_H
#define DIFFBOT_IMU_CONFIG_H

#include <stdint.h>

namespace diffbot
{
    /**
     * @brief The settings of \ref ImuPublisher: whether it runs, its timing and the values it puts
     * in the message.
     *
     * One typed value that the publisher's constructor takes, instead of preprocessor
     * definitions: the compiler checks the types, \ref valid checks the values, and the unit tests
     * pass settings of their own. The robot's values are in diffbot_base_config.h. They are
     * compiled in, so a change still needs a new build of the firmware.
     */
    struct ImuPublisherConfig
    {
        /// false for a robot without an IMU: no topic, no I2C traffic
        bool enabled;
        uint32_t period_ms;                  ///< Sampling period, for example 20 ms for 50 Hz
        uint32_t retry_ms;                   ///< Look for a missing or failed IMU again after this
        uint32_t read_budget_us;             ///< Drop a sample whose read took longer
        /// header.frame_id: the robot model's link of the sensor
        const char* frame_id;
        float angular_velocity_variance;     ///< (rad/s)², on the covariance's diagonal
        float linear_acceleration_variance;  ///< (m/s²)², on the covariance's diagonal

        /// True if the publisher can work with these values: a period, a read budget shorter than
        /// the period, a retry time, a frame, and variances that are numbers of at least zero
        constexpr bool valid() const
        {
            // 64 bits, so that a long period doesn't wrap: unsigned long has 32 bits on the Teensy
            return period_ms > 0 && read_budget_us > 0
                && read_budget_us < uint64_t(period_ms) * 1000ULL
                && retry_ms > 0 && frame_id != nullptr && frame_id[0] != '\0'
                && angular_velocity_variance >= 0.0f && angular_velocity_variance < 1.0e6f
                && linear_acceleration_variance >= 0.0f && linear_acceleration_variance < 1.0e6f;
        }
    };
}

#endif // DIFFBOT_IMU_CONFIG_H
