#ifndef DIFFBOT_BNO055_H
#define DIFFBOT_BNO055_H

#include <stdint.h>

#include "i2c_bus.h"

namespace diffbot
{
    /**
     * @brief One sample of the IMU, in REP 103 units and in the sensor's own axes.
     *
     * The rates and accelerations come from one burst read, so they belong to the same moment.
     */
    struct ImuSample
    {
        float angular_velocity[3];    ///< rad/s, about the sensor's x, y and z axes
        float linear_acceleration[3]; ///< m/s², the raw specific force, including gravity
        uint8_t calibration;          ///< CALIB_STAT: system, gyroscope, accelerometer, magnetometer (2 bits each)
    };

    /**
     * @brief Converts the BNO055's 18 data bytes from ACC_DATA_X_LSB (0x08) to GYR_DATA_Z_MSB (0x19)
     * into a sample. The magnetometer bytes in between are ignored.
     *
     * \ref Bno055 selects m/s² (100 LSB per m/s²) and rad/s (900 LSB per rad/s) in UNIT_SEL.
     */
    void bno055Convert(const uint8_t data[18], ImuSample& sample);

    /**
     * @brief Reads Bosch's BNO055 over I2C, in its IMU fusion mode (IMUPLUS): accelerometer and
     * gyroscope, without the magnetometer.
     *
     * Only the raw angular velocity and acceleration are read, not the chip's own orientation
     * estimate: the EKF on the robot's computer fuses the angular velocity. The chip's axes are left
     * as mounted (the default axis map is written explicitly); the URDF's imu_link describes how it
     * sits on the robot.
     *
     * The start-up is a state machine that \ref update drives without waiting: each call does at
     * most a few short transfers and returns, so the firmware's control loop keeps running while
     * the chip resets. Every transfer is checked, so that \ref read returns false instead of old
     * values when the bus or the sensor fails, and a sensor that reset itself (for example after a
     * brown-out) goes back through the start-up.
     *
     * Register addresses and timings from Bosch's BNO055 datasheet (BST-BNO055-DS000, rev. 1.8).
     */
    class Bno055
    {
    public:
        enum State
        {
            STARTING,         ///< About to check the chip ID
            WAITING_CONFIG,   ///< In the switch to configuration mode (19 ms)
            WAITING_RESET,    ///< Reset sent; polling the chip ID until it answers (650 ms typical)
            WAITING_MODE,     ///< Switching to IMUPLUS (7 ms)
            WAITING_FUSION,   ///< Polling SYS_STATUS until the fusion runs
            READY,            ///< Samples can be read
            ABSENT,           ///< No BNO055 answered at the address
            FAILED            ///< It answered, but didn't come back after the reset, didn't take a setting or didn't start the fusion
        };

        static const uint8_t ADDRESS_DEFAULT = 0x28;   ///< COM3/ADR pin low
        static const uint8_t ADDRESS_ALTERNATE = 0x29; ///< COM3/ADR pin high

        Bno055(I2cBus& bus, uint8_t address = ADDRESS_DEFAULT);

        /// Starts the start-up again, from the chip ID check
        void restart();

        /**
         * @brief Advances the start-up, if it isn't finished.
         * @param now_ms A monotonic time in milliseconds, for example millis()
         * @return The state after this step
         */
        State update(uint32_t now_ms);

        State state() const { return state_; }

        /**
         * @brief Reads one sample, if \ref state is READY.
         *
         * Besides the data, it reads the status registers. A sample counts only while SYS_STATUS
         * says that the fusion runs: in its other states (idle, initializing, self-test) the data
         * registers may hold old values. If the operating mode or the units aren't what the
         * start-up set, the sensor has reset itself; then, or after a system error, the state goes
         * back to STARTING.
         * @return false if not ready, if a transfer failed, if the fusion doesn't run or if the
         * sensor reset itself; \p sample is then unchanged.
         */
        bool read(ImuSample& sample);

        // Registers of page 0
        static const uint8_t REG_CHIP_ID = 0x00;
        static const uint8_t REG_PAGE_ID = 0x07;
        static const uint8_t REG_ACC_DATA_X_LSB = 0x08;
        static const uint8_t REG_CALIB_STAT = 0x35;
        static const uint8_t REG_SYS_STATUS = 0x39;
        static const uint8_t REG_UNIT_SEL = 0x3B;
        static const uint8_t REG_OPR_MODE = 0x3D;
        static const uint8_t REG_PWR_MODE = 0x3E;
        static const uint8_t REG_SYS_TRIGGER = 0x3F;
        static const uint8_t REG_AXIS_MAP_CONFIG = 0x41;
        static const uint8_t REG_AXIS_MAP_SIGN = 0x42;

        static const uint8_t CHIP_ID = 0xA0;
        static const uint8_t MODE_CONFIG = 0x00;
        static const uint8_t MODE_IMUPLUS = 0x08;
        /// m/s² for the accelerometer (bit 0 clear), rad/s for the gyroscope (bit 1 set)
        static const uint8_t UNITS = 0x02;
        static const uint8_t AXIS_MAP_CONFIG_DEFAULT = 0x24;
        static const uint8_t AXIS_MAP_SIGN_DEFAULT = 0x00;
        static const uint8_t SYS_STATUS_ERROR = 0x01;
        static const uint8_t SYS_STATUS_FUSION_RUNNING = 0x05;
        static const uint8_t DATA_LENGTH = 18;
        /// CALIB_STAT (0x35) to UNIT_SEL (0x3B), read with each sample. OPR_MODE (0x3D) is read on
        /// its own, to skip the reserved register 0x3C, as the datasheet advises (section 4.1)
        static const uint8_t STATUS_LENGTH = 7;

        static const uint32_t CONFIG_MODE_MS = 25;     ///< 19 ms from an operation mode
        static const uint32_t RESET_FIRST_POLL_MS = 650;
        static const uint32_t RESET_POLL_MS = 50;
        static const uint32_t RESET_TIMEOUT_MS = 2000;
        static const uint32_t IMUPLUS_MODE_MS = 20;    ///< 7 ms from configuration mode
        static const uint32_t FUSION_POLL_MS = 10;
        static const uint32_t FUSION_TIMEOUT_MS = 1000; ///< From the switch to IMUPLUS

    private:
        bool configure();
        void enter(State state, uint32_t now_ms);

        I2cBus& bus_;
        uint8_t address_;
        State state_;
        uint32_t step_started_ms_;
        uint32_t next_step_ms_; ///< In WAITING_RESET and WAITING_FUSION: when to poll next, relative to step_started_ms_
    };
}

#endif // DIFFBOT_BNO055_H
