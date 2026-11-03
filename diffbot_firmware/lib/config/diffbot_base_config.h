#ifndef DIFFBOT_BASE_CONFIG_H
#define DIFFBOT_BASE_CONFIG_H



/// Encoder pins
#define ENCODER_LEFT_H1 5
#define ENCODER_LEFT_H2 6
// Encoder resolution used for initialization 
// will be read from parameter server
#define ENCODER_RESOLUTION 542

#define ENCODER_RIGHT_H1 7
#define ENCODER_RIGHT_H2 8

/// Motor i2c address
#define MOTOR_DRIVER_ADDR 0x60
#define MOTOR_LEFT 4
#define MOTOR_RIGHT 3

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

#define PWM_BITS 8  // PWM Resolution of the microcontroller


#define UPDATE_RATE_CONTROL 20
// BaseController keeps an IMU period from this rate, which nothing uses. The IMU publisher's
// period is in IMU_PUBLISHER_CONFIG below.
#define UPDATE_RATE_IMU 1
#define UPDATE_RATE_DEBUG 5

#define E_STOP_COMMAND_RECEIVED_DURATION 5 // Set the target velocities to zero if no command arrived for this many seconds (#146)

/// IMU: Bosch BNO055 on the Teensy's I2C bus (Wire, the same bus as the motor driver)
#include "imu_config.h"

/// The BNO055's I2C address: 0x28, or 0x29 if the breakout's ADR pin is high
constexpr uint8_t IMU_BNO055_ADDRESS = 0x28;

/// The IMU publisher's settings (lib/imu/imu_config.h), in the order of its fields.
/// - enabled: false for a robot without an IMU.
/// - period_ms: 20 ms, 50 Hz. The message, sensor_msgs/Imu with frame imu_link (about 330 bytes
///   with rosserial's framing), fits the 512-byte rosserial buffers of both Teensy boards at
///   50 Hz over USB serial.
/// - retry_ms: look for a missing IMU again after 10 s.
/// - read_budget_us: drop a sample whose read took longer than 10 ms, half the period.
/// - frame_id: imu_link, the link of the sensor in the robot model.
/// - The variances, (rad/s)² and (m/s²)²: starting values, not measured yet. They are chosen
///   standard deviations of 0.02 rad/s and 0.1 m/s², squared. For scale, from the BNO055
///   datasheet (rev. 1.8, table 0-2, and table 3-7 for the bandwidths in the fusion modes): the
///   gyroscope's noise of 0.014 °/s/√Hz gives about 0.08 °/s (0.0014 rad/s) rms at its 32 Hz
///   bandwidth. Its zero-rate offset is typically ±1 °/s (0.017 rad/s), specified with the
///   offset cancellation off. The accelerometer's 150 µg/√Hz give about 1.2 mg (0.012 m/s²) rms
///   at 62.5 Hz, and its offset is typically ±80 mg. The gyroscope's value covers its typical
///   offset as well, so that the EKF doesn't take a biased rate as exact. Replace both with the
///   variance and the bias of a stationary recording at this rate (lib/imu/README.md, bench
///   checks).
constexpr diffbot::ImuPublisherConfig IMU_PUBLISHER_CONFIG = {
    true,       // enabled
    20,         // period_ms
    10000,      // retry_ms
    10000,      // read_budget_us
    "imu_link", // frame_id
    4.0e-4f,    // angular_velocity_variance, (0.02 rad/s)²
    1.0e-2f,    // linear_acceleration_variance, (0.1 m/s²)²
};
static_assert(IMU_PUBLISHER_CONFIG.valid(),
              "IMU_PUBLISHER_CONFIG: check its values (lib/imu/imu_config.h)");



#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -(PWM_MAX)

#endif // DIFFBOT_BASE_CONFIG_H
