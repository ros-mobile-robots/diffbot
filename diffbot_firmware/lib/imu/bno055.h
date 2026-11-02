#ifndef DIFFBOT_BNO055_H
#define DIFFBOT_BNO055_H

#include <stdint.h>

#include "i2c_bus.h"

namespace diffbot
{
    /**
     * @brief One sample of the IMU, in REP 103 units and in the sensor's own axes.
     *
     * The rates and accelerations are read in one burst. Register shadowing prevents mixing
     * an axis's low and high bytes from different updates (datasheet section 3.7).
     * The datasheet does not specify that all six values share a measurement time.
     *
     * In fusion mode, the chip corrects both sensors' outputs for their offsets
     * (datasheet sections 3.6.5.1 and 3.6.5.3). They aren't the sensors' uncorrected output.
     * They aren't bias-free either.
     */
    struct ImuSample
    {
        /// rad/s, about the sensor's x, y and z axes, offset-corrected
        float angular_velocity[3];
        /// m/s², the specific force including gravity, offset-corrected
        float linear_acceleration[3];
        uint8_t calibration;          ///< CALIB_STAT: system, gyroscope, accelerometer, magnetometer (2 bits each)
    };

    /**
     * @brief Converts the BNO055's 18 data bytes from ACC_DATA_X_LSB (0x08) to GYR_DATA_Z_MSB (0x19)
     * into a sample. The magnetometer bytes in between are ignored.
     *
     * It sets only the angular velocity and the acceleration. The calibration stays as it was,
     * for the caller to fill in.
     *
     * \ref Bno055 selects m/s² (100 LSB per m/s²) and rad/s (900 LSB per rad/s) in UNIT_SEL.
     */
    void bno055Convert(const uint8_t data[18], ImuSample& sample);

    /**
     * @brief Reads Bosch's BNO055 over I2C, in its IMU fusion mode: accelerometer and gyroscope,
     * without the magnetometer.
     *
     * In IMU mode (called IMUPLUS by Bosch's and Adafruit's APIs), the chip's own processor runs
     * Bosch's sensor fusion: it computes a relative orientation from the accelerometer and the
     * gyroscope (datasheet section 3.3.3.1), calibrates them in the background, and corrects the
     * data registers for their offsets (sections 3.6.5.1 and 3.6.5.3). This driver reads only
     * those corrected angular velocities and accelerations, not the orientation: the EKF on the
     * robot's computer fuses the angular velocity with the wheel odometry. A mode without fusion
     * (AMG or ACCGYRO) would give uncorrected values. The calibration status (CALIB_STAT) comes
     * with each sample, but doesn't hold samples back.
     *
     * The chip's axes are left as mounted (the default axis map is written explicitly). The URDF's
     * imu_link describes how it sits on the robot.
     *
     * The chip runs on its internal oscillator, its default (datasheet section 5.5). The datasheet
     * recommends an external 32.768 kHz crystal for the best performance. Switching to one takes
     * at least about 600 ms and a check that the chip didn't fall back to the internal clock,
     * so the driver selects the internal one and checks that it is selected.
     *
     * The start-up is a state machine that \ref update drives without waiting for the chip: each
     * call does the work that is due and returns, so the loop can run between start-up steps, for
     * example while the chip resets. Most calls make two transfers or fewer. The call that
     * configures the chip writes its settings and reads them back. The transfers themselves are
     * synchronous. The sample transfers and the configuration's read-back are checked, so that
     * \ref read returns false instead of old values when the bus or the sensor fails, and a sensor
     * that reset itself (for example after a brown-out) goes back through the start-up.
     *
     * Register addresses and timings from Bosch's BNO055 datasheet (BST-BNO055-DS000, rev. 1.8).
     */
    class Bno055
    {
    public:
        /// Driver start-up states, not sensor operating modes (OPR_MODE)
        enum State
        {
            STARTING,         ///< About to check the chip ID
            WAITING_CONFIG,   ///< In the switch to configuration mode (19 ms)
            WAITING_RESET,    ///< Reset sent; polling the chip ID until it answers (650 ms typical)
            WAITING_IMU_MODE, ///< In the switch to IMU mode (7 ms)
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
        static const uint8_t PAGE_ZERO = 0x00;
        static const uint8_t MODE_CONFIG = 0x00;
        /// Datasheet IMU mode (section 3.3.3.1), called IMUPLUS by Bosch's and Adafruit's APIs
        static const uint8_t MODE_IMU = 0x08;
        static const uint8_t POWER_NORMAL = 0x00;
        /// m/s² for the accelerometer (bit 0 clear), rad/s for the gyroscope (bit 1 set)
        static const uint8_t UNITS = 0x02;
        static const uint8_t AXIS_MAP_CONFIG_DEFAULT = 0x24;
        static const uint8_t AXIS_MAP_SIGN_DEFAULT = 0x00;
        /// SYS_TRIGGER's RST_SYS bit, write-only: resets the chip (datasheet section 4.3.63)
        static const uint8_t SYS_TRIGGER_RESET = 1u << 5;
        /// SYS_TRIGGER's CLK_SEL bit, readable: set for an external crystal, clear for the
        /// internal oscillator
        static const uint8_t SYS_TRIGGER_CLK_SEL = 1u << 7;
        /// SYS_TRIGGER with CLK_SEL clear, for the internal oscillator, and no command bit set
        static const uint8_t SYS_TRIGGER_INTERNAL_CLOCK = 0x00;
        static const uint8_t SYS_STATUS_ERROR = 0x01;
        static const uint8_t SYS_STATUS_FUSION_RUNNING = 0x05;
        static const uint8_t DATA_LENGTH = 18;
        /// CALIB_STAT (0x35) to UNIT_SEL (0x3B), read with each sample. OPR_MODE (0x3D) is read on
        /// its own, to skip the reserved register 0x3C, as the datasheet advises (section 4.1)
        static const uint8_t STATUS_LENGTH = 7;

        /// Wait after the configuration mode request: the switch takes 19 ms from another mode
        static const uint32_t CONFIG_SETTLE_MS = 25;
        static const uint32_t RESET_FIRST_POLL_MS = 650;
        static const uint32_t RESET_POLL_MS = 50;
        /// From the reset request. The two timeouts are deadlines: the first call from a deadline
        /// on fails the start-up without another poll, also when the call comes late
        static const uint32_t RESET_TIMEOUT_MS = 2000;
        /// Wait after the IMU mode request: the switch takes 7 ms from configuration mode
        static const uint32_t IMU_SETTLE_MS = 20;
        static const uint32_t FUSION_POLL_MS = 10;
        static const uint32_t FUSION_TIMEOUT_MS = 1000; ///< From the IMU mode request

    private:
        // One handler per start-up state: each does the work that is due now and returns
        void checkChipAndEnterConfig(uint32_t now_ms);
        void resetAfterConfigSettles(uint32_t now_ms);
        void pollResetCompletion(uint32_t now_ms);
        void waitForImuMode(uint32_t now_ms);
        void pollFusionStatus(uint32_t now_ms);

        /// Sets the state, and nothing else: a timed phase that goes on keeps its time origin
        void transitionTo(State next);
        /// Sets the state and starts a timed phase at \p now_ms, with the first poll due at once
        void startTimedPhase(State next, uint32_t now_ms);
        /// Milliseconds since the timed phase began, also when millis() wrapped in between
        uint32_t phaseElapsedMs(uint32_t now_ms) const;

        bool chipAnswers();
        void requestReset();
        void configureAndRequestImuMode(uint32_t now_ms);

        /// Writes the settings, reads them back and checks them
        bool configure();
        bool selectPageZero();
        bool writeSettings();
        bool selectInternalClock();
        bool verifySettings();
        bool verifyInternalClock();

        bool readRegister(uint8_t reg, uint8_t& value);

        I2cBus& bus_;
        uint8_t address_;
        State state_;
        /// When the current timed phase began. WAITING_IMU_MODE and WAITING_FUSION are one phase,
        /// from the IMU mode request, so that the fusion's deadline counts from that request
        uint32_t phase_started_ms_;
        /// In WAITING_RESET and WAITING_FUSION: when to poll next, in ms after phase_started_ms_
        uint32_t next_poll_elapsed_ms_;
    };
}

#endif // DIFFBOT_BNO055_H
