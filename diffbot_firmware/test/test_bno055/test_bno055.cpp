// Unit tests of the BNO055 driver (lib/imu), on the PC without hardware: a fake I2C bus stands in
// for the sensor. Run them with:
//   pio test -e native
#include <unity.h>

#include <stdint.h>

#include <initializer_list>

#include "bno055.h"
#include "../fakes/fake_bno055.h"

using namespace diffbot;

typedef FakeBno055Bus FakeBus;

// Calls update every millisecond until the given state or a time limit, and returns the time.
// It counts the time as the driver does, unsigned, so it also runs across the rollover of millis()
uint32_t runUntil(Bno055& imu, Bno055::State wanted, uint32_t start_ms, uint32_t limit_ms)
{
    uint32_t t = start_ms;
    while (t - start_ms < limit_ms && imu.update(t) != wanted)
        ++t;
    return t;
}

void test_bno055_convert_scales_and_signs_the_data_bytes()
{
    uint8_t data[18] = {0};
    setLe16(&data[0], 100);    // accelerometer x: 1 m/s²
    setLe16(&data[2], -50);    // y: -0.5 m/s²
    setLe16(&data[4], 981);    // z: gravity, 9.81 m/s²
    setLe16(&data[6], 1234);   // magnetometer, ignored
    setLe16(&data[12], 900);   // gyroscope x: 1 rad/s
    setLe16(&data[14], -450);  // y: -0.5 rad/s
    setLe16(&data[16], 32767); // z: the largest value, 36.4 rad/s
    ImuSample sample;
    bno055Convert(data, sample);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, sample.linear_acceleration[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f, sample.linear_acceleration[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 9.81f, sample.linear_acceleration[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, sample.angular_velocity[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f, sample.angular_velocity[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 32767.0f / 900.0f, sample.angular_velocity[2]);
}

void test_bno055_startup_advances_one_step_per_update_without_waiting()
{
    FakeBus bus;
    Bno055 imu(bus);
    TEST_ASSERT_EQUAL(Bno055::WAITING_CONFIG, imu.update(1000));
    // Nothing happens before the configuration mode switch has had its time
    TEST_ASSERT_EQUAL(Bno055::WAITING_CONFIG, imu.update(1000 + Bno055::CONFIG_SETTLE_MS - 1));
    TEST_ASSERT_EQUAL(Bno055::WAITING_RESET, imu.update(1000 + Bno055::CONFIG_SETTLE_MS));
    TEST_ASSERT_EQUAL(1, bus.reset_count);
    bus.resetting = false; // the chip is back
    uint32_t t = runUntil(imu, Bno055::READY, 1000 + Bno055::CONFIG_SETTLE_MS, 3000);
    TEST_ASSERT_EQUAL(Bno055::READY, imu.state());
    // The first poll after the reset waits its typical 650 ms. Then 20 ms for IMU mode, and the
    // fusion runs at the first poll of SYS_STATUS
    TEST_ASSERT_TRUE(t - 1000 >= Bno055::CONFIG_SETTLE_MS + Bno055::RESET_FIRST_POLL_MS
                                     + Bno055::IMU_SETTLE_MS);
    TEST_ASSERT_EQUAL_HEX8(Bno055::UNITS, bus.registers[Bno055::REG_UNIT_SEL]);
    TEST_ASSERT_EQUAL_HEX8(Bno055::MODE_IMU, bus.registers[Bno055::REG_OPR_MODE]);
    TEST_ASSERT_EQUAL_HEX8(Bno055::AXIS_MAP_CONFIG_DEFAULT, bus.registers[Bno055::REG_AXIS_MAP_CONFIG]);
    TEST_ASSERT_EQUAL_HEX8(Bno055::AXIS_MAP_SIGN_DEFAULT, bus.registers[Bno055::REG_AXIS_MAP_SIGN]);
}

void test_bno055_startup_reports_absent_without_a_chip_at_the_address()
{
    FakeBus wrong_chip;
    wrong_chip.registers[Bno055::REG_CHIP_ID] = 0x00;
    Bno055 imu(wrong_chip);
    TEST_ASSERT_EQUAL(Bno055::ABSENT, imu.update(0));

    FakeBus silent;
    silent.fail_reads = true;
    Bno055 missing(silent);
    TEST_ASSERT_EQUAL(Bno055::ABSENT, missing.update(0));

    FakeBus other_address;
    Bno055 wrong(other_address, Bno055::ADDRESS_ALTERNATE);
    TEST_ASSERT_EQUAL(Bno055::ABSENT, wrong.update(0));
}

void test_bno055_startup_fails_when_the_chip_stays_silent_after_the_reset()
{
    FakeBus bus;
    bus.back_after_reset = false;
    Bno055 imu(bus);
    uint32_t t = runUntil(imu, Bno055::FAILED, 0, 5000);
    TEST_ASSERT_EQUAL(Bno055::FAILED, imu.state());
    TEST_ASSERT_TRUE(t <= Bno055::CONFIG_SETTLE_MS + Bno055::RESET_TIMEOUT_MS
                              + Bno055::RESET_POLL_MS + 1);
}

void test_bno055_startup_fails_when_a_setting_reads_back_different()
{
    FakeBus bus;
    bus.ignore_writes_to = Bno055::REG_UNIT_SEL;
    Bno055 imu(bus);
    imu.update(0);
    imu.update(Bno055::CONFIG_SETTLE_MS);
    bus.resetting = false;
    runUntil(imu, Bno055::FAILED, Bno055::CONFIG_SETTLE_MS, 3000);
    TEST_ASSERT_EQUAL(Bno055::FAILED, imu.state());
}

void test_bno055_startup_fails_when_the_page_or_the_clock_reads_back_different()
{
    // The chip stays on page 1
    FakeBus page_one;
    page_one.ignore_writes_to = Bno055::REG_PAGE_ID;
    page_one.registers[Bno055::REG_PAGE_ID] = 0x01;
    Bno055 imu(page_one);
    runUntil(imu, Bno055::FAILED, 0, 3000);
    TEST_ASSERT_EQUAL(Bno055::FAILED, imu.state());

    // The chip keeps the external clock selected
    FakeBus external_clock;
    external_clock.ignore_writes_to = Bno055::REG_SYS_TRIGGER;
    external_clock.registers[Bno055::REG_SYS_TRIGGER] = Bno055::SYS_TRIGGER_CLK_SEL;
    Bno055 clocked(external_clock);
    runUntil(clocked, Bno055::FAILED, 0, 3000);
    TEST_ASSERT_EQUAL(Bno055::FAILED, clocked.state());
}

void test_bno055_startup_fails_when_the_page_or_the_clock_read_back_fails()
{
    for (int reg : {Bno055::REG_PAGE_ID, Bno055::REG_SYS_TRIGGER})
    {
        FakeBus bus;
        bus.fail_reads_starting_at = reg;
        Bno055 imu(bus);
        runUntil(imu, Bno055::FAILED, 0, 3000);
        TEST_ASSERT_EQUAL(Bno055::FAILED, imu.state());
    }
}

// Runs the start-up of a test's driver until it's ready: the chip answers after its reset
void startReady(Bno055& imu, FakeBus& bus)
{
    imu.update(0);
    imu.update(Bno055::CONFIG_SETTLE_MS);
    bus.resetting = false;
    runUntil(imu, Bno055::READY, Bno055::CONFIG_SETTLE_MS, 3000);
    TEST_ASSERT_EQUAL(Bno055::READY, imu.state());
}

void test_bno055_read_fills_the_sample_when_ready()
{
    FakeBus bus;
    Bno055 imu(bus);
    startReady(imu, bus);
    setLe16(&bus.registers[Bno055::REG_ACC_DATA_X_LSB + 4], 981);
    setLe16(&bus.registers[Bno055::REG_ACC_DATA_X_LSB + 16], 90);
    bus.registers[Bno055::REG_CALIB_STAT] = 0x3C;
    ImuSample sample;
    TEST_ASSERT_TRUE(imu.read(sample));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 9.81f, sample.linear_acceleration[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, sample.angular_velocity[2]);
    TEST_ASSERT_EQUAL_HEX8(0x3C, sample.calibration);
}

void test_bno055_read_fails_before_the_startup_is_done()
{
    FakeBus bus;
    Bno055 imu(bus);
    ImuSample sample;
    TEST_ASSERT_FALSE(imu.read(sample));
}

void test_bno055_read_leaves_the_sample_unchanged_when_a_transfer_fails()
{
    FakeBus bus;
    Bno055 imu(bus);
    startReady(imu, bus);
    bus.fail_reads = true;
    ImuSample sample;
    sample.angular_velocity[2] = 42.0f;
    TEST_ASSERT_FALSE(imu.read(sample));
    TEST_ASSERT_EQUAL_FLOAT(42.0f, sample.angular_velocity[2]);
    TEST_ASSERT_EQUAL(Bno055::READY, imu.state());
}

void test_bno055_read_leaves_the_whole_sample_unchanged_when_a_status_read_fails()
{
    // The motion data arrives, then the status block's or the mode's read fails
    for (int reg : {Bno055::REG_CALIB_STAT, Bno055::REG_OPR_MODE})
    {
        FakeBus bus;
        Bno055 imu(bus);
        startReady(imu, bus);
        setLe16(&bus.registers[Bno055::REG_ACC_DATA_X_LSB], 981);
        setLe16(&bus.registers[Bno055::REG_ACC_DATA_X_LSB + 12], 900);
        bus.fail_reads_starting_at = reg;
        const ImuSample before = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, 0x5A};
        ImuSample sample = before;
        TEST_ASSERT_FALSE(imu.read(sample));
        TEST_ASSERT_EQUAL(1, bus.data_reads);
        TEST_ASSERT_EQUAL_FLOAT_ARRAY(before.angular_velocity, sample.angular_velocity, 3);
        TEST_ASSERT_EQUAL_FLOAT_ARRAY(before.linear_acceleration, sample.linear_acceleration, 3);
        TEST_ASSERT_EQUAL_HEX8(before.calibration, sample.calibration);
        TEST_ASSERT_EQUAL(Bno055::READY, imu.state());
    }
}

void test_bno055_read_restarts_the_startup_after_a_sensor_reset()
{
    FakeBus bus;
    Bno055 imu(bus);
    startReady(imu, bus);
    // A brown-out: the sensor is back in configuration mode with its default units
    bus.registers[Bno055::REG_OPR_MODE] = Bno055::MODE_CONFIG;
    ImuSample sample;
    TEST_ASSERT_FALSE(imu.read(sample));
    TEST_ASSERT_EQUAL(Bno055::STARTING, imu.state());
}

void test_bno055_read_restarts_the_startup_after_a_system_error()
{
    FakeBus bus;
    Bno055 imu(bus);
    startReady(imu, bus);
    bus.registers[Bno055::REG_SYS_STATUS] = Bno055::SYS_STATUS_ERROR;
    ImuSample sample;
    TEST_ASSERT_FALSE(imu.read(sample));
    TEST_ASSERT_EQUAL(Bno055::STARTING, imu.state());
}

void test_bno055_startup_waits_until_the_fusion_runs()
{
    FakeBus bus;
    bus.fusion_status = 0x03; // system initialization
    Bno055 imu(bus);
    uint32_t t = runUntil(imu, Bno055::WAITING_FUSION, 0, 3000);
    TEST_ASSERT_EQUAL(Bno055::WAITING_FUSION, imu.state());
    t = runUntil(imu, Bno055::READY, t, 500);
    TEST_ASSERT_EQUAL(Bno055::WAITING_FUSION, imu.state());
    bus.registers[Bno055::REG_SYS_STATUS] = Bno055::SYS_STATUS_FUSION_RUNNING;
    runUntil(imu, Bno055::READY, t, Bno055::FUSION_POLL_MS + 1);
    TEST_ASSERT_EQUAL(Bno055::READY, imu.state());
}

void test_bno055_startup_fails_when_the_fusion_doesnt_start()
{
    FakeBus bus;
    bus.fusion_status = 0x03;
    Bno055 imu(bus);
    uint32_t t = runUntil(imu, Bno055::WAITING_FUSION, 0, 3000);
    uint32_t failed = runUntil(imu, Bno055::FAILED, t, 3000);
    TEST_ASSERT_EQUAL(Bno055::FAILED, imu.state());
    // FUSION_TIMEOUT_MS after the switch to IMU mode, which was IMU_SETTLE_MS before t
    TEST_ASSERT_TRUE(failed - t + Bno055::IMU_SETTLE_MS
                     <= Bno055::FUSION_TIMEOUT_MS + Bno055::FUSION_POLL_MS + 1);

    FakeBus error;
    error.fusion_status = Bno055::SYS_STATUS_ERROR;
    Bno055 broken(error);
    runUntil(broken, Bno055::FAILED, 0, 3000);
    TEST_ASSERT_EQUAL(Bno055::FAILED, broken.state());
}

void test_bno055_startup_counts_the_fusion_timeout_from_the_imu_mode_request()
{
    // Waiting for IMU mode and polling SYS_STATUS are one timed phase. It starts just before
    // millis() rolls over, and its deadline comes after the rollover
    FakeBus bus;
    bus.fusion_status = 0x03; // system initialization, the fusion doesn't start
    Bno055 imu(bus);
    const uint32_t start_ms = UINT32_MAX - 800;
    imu.update(start_ms);
    imu.update(start_ms + Bno055::CONFIG_SETTLE_MS);
    bus.resetting = false;
    const uint32_t mode_request_ms =
        start_ms + Bno055::CONFIG_SETTLE_MS + Bno055::RESET_FIRST_POLL_MS;
    TEST_ASSERT_EQUAL(Bno055::WAITING_IMU_MODE, imu.update(mode_request_ms));
    uint32_t failed_ms = runUntil(imu, Bno055::FAILED, mode_request_ms + 1, 3000);
    TEST_ASSERT_EQUAL(Bno055::FAILED, imu.state());
    // At the first poll from the deadline on. From the switch to WAITING_FUSION, it would be
    // IMU_SETTLE_MS later
    const uint32_t failed_after_ms = failed_ms - mode_request_ms;
    TEST_ASSERT_TRUE(failed_after_ms >= Bno055::FUSION_TIMEOUT_MS);
    TEST_ASSERT_TRUE(failed_after_ms <= Bno055::FUSION_TIMEOUT_MS + Bno055::FUSION_POLL_MS);
}

void test_bno055_read_fails_while_the_fusion_doesnt_run()
{
    // Idle, initializing peripherals, initializing, self-test, running without fusion: the data
    // registers may be old
    FakeBus bus;
    Bno055 imu(bus);
    startReady(imu, bus);
    ImuSample sample;
    for (uint8_t status : {0x00, 0x02, 0x03, 0x04, 0x06})
    {
        bus.registers[Bno055::REG_SYS_STATUS] = status;
        TEST_ASSERT_FALSE(imu.read(sample));
        TEST_ASSERT_EQUAL(Bno055::READY, imu.state());
    }
    bus.registers[Bno055::REG_SYS_STATUS] = Bno055::SYS_STATUS_FUSION_RUNNING;
    TEST_ASSERT_TRUE(imu.read(sample));
}

void test_bno055_read_skips_the_reserved_register()
{
    FakeBus bus;
    Bno055 imu(bus);
    startReady(imu, bus);
    ImuSample sample;
    TEST_ASSERT_TRUE(imu.read(sample));
    TEST_ASSERT_EQUAL(0, bus.reserved_reads);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_bno055_convert_scales_and_signs_the_data_bytes);
    RUN_TEST(test_bno055_startup_advances_one_step_per_update_without_waiting);
    RUN_TEST(test_bno055_startup_reports_absent_without_a_chip_at_the_address);
    RUN_TEST(test_bno055_startup_fails_when_the_chip_stays_silent_after_the_reset);
    RUN_TEST(test_bno055_startup_fails_when_a_setting_reads_back_different);
    RUN_TEST(test_bno055_startup_fails_when_the_page_or_the_clock_reads_back_different);
    RUN_TEST(test_bno055_startup_fails_when_the_page_or_the_clock_read_back_fails);
    RUN_TEST(test_bno055_read_fills_the_sample_when_ready);
    RUN_TEST(test_bno055_read_fails_before_the_startup_is_done);
    RUN_TEST(test_bno055_read_leaves_the_sample_unchanged_when_a_transfer_fails);
    RUN_TEST(test_bno055_read_leaves_the_whole_sample_unchanged_when_a_status_read_fails);
    RUN_TEST(test_bno055_read_restarts_the_startup_after_a_sensor_reset);
    RUN_TEST(test_bno055_read_restarts_the_startup_after_a_system_error);
    RUN_TEST(test_bno055_startup_waits_until_the_fusion_runs);
    RUN_TEST(test_bno055_startup_fails_when_the_fusion_doesnt_start);
    RUN_TEST(test_bno055_startup_counts_the_fusion_timeout_from_the_imu_mode_request);
    RUN_TEST(test_bno055_read_fails_while_the_fusion_doesnt_run);
    RUN_TEST(test_bno055_read_skips_the_reserved_register);
    return UNITY_END();
}
