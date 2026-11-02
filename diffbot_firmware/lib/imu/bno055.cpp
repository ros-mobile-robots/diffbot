#include "bno055.h"

namespace
{
    using diffbot::Bno055;

    /// One register and the value that the start-up writes to it and reads back
    struct RegisterSetting
    {
        uint8_t reg;
        uint8_t value;
    };

    /// The ordinary settings, written explicitly instead of relying on the reset, in this order.
    /// The page and the clock aren't in it: they are written and checked on their own
    const RegisterSetting SETTINGS[] = {
        {Bno055::REG_PWR_MODE, Bno055::POWER_NORMAL},
        {Bno055::REG_UNIT_SEL, Bno055::UNITS},
        {Bno055::REG_AXIS_MAP_CONFIG, Bno055::AXIS_MAP_CONFIG_DEFAULT},
        {Bno055::REG_AXIS_MAP_SIGN, Bno055::AXIS_MAP_SIGN_DEFAULT},
    };

    /// Decodes a signed 16-bit value from its two bytes, least significant first. Combining the
    /// bytes, instead of casting the buffer, makes no assumption on the alignment or byte order
    int16_t decodeSignedLe16(const uint8_t* bytes)
    {
        return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
    }
}

void diffbot::bno055Convert(const uint8_t data[18], ImuSample& sample)
{
    // Bytes 0-5: accelerometer x, y, z; 6-11: magnetometer (unused); 12-17: gyroscope x, y, z
    for (int i = 0; i < 3; ++i)
    {
        sample.linear_acceleration[i] = decodeSignedLe16(&data[2 * i]) / 100.0f;
        sample.angular_velocity[i] = decodeSignedLe16(&data[12 + 2 * i]) / 900.0f;
    }
}

diffbot::Bno055::Bno055(I2cBus& bus, uint8_t address)
    : bus_(bus), address_(address), state_(STARTING), phase_started_ms_(0),
      next_poll_elapsed_ms_(0)
{
}

void diffbot::Bno055::restart()
{
    transitionTo(STARTING);
}

diffbot::Bno055::State diffbot::Bno055::update(uint32_t now_ms)
{
    // One state per call: its handler does what is due now and returns
    switch (state_)
    {
    case STARTING:
        checkChipAndEnterConfig(now_ms);
        break;
    case WAITING_CONFIG:
        resetAfterConfigSettles(now_ms);
        break;
    case WAITING_RESET:
        pollResetCompletion(now_ms);
        break;
    case WAITING_IMU_MODE:
        waitForImuMode(now_ms);
        break;
    case WAITING_FUSION:
        pollFusionStatus(now_ms);
        break;
    case READY:
    case ABSENT:
    case FAILED:
        break;
    }
    return state_;
}

void diffbot::Bno055::checkChipAndEnterConfig(uint32_t now_ms)
{
    if (!chipAnswers())
    {
        transitionTo(ABSENT);
        return;
    }
    // Settings can only change in configuration mode
    if (!bus_.writeRegister(address_, REG_OPR_MODE, MODE_CONFIG))
    {
        transitionTo(FAILED);
        return;
    }
    startTimedPhase(WAITING_CONFIG, now_ms);
}

void diffbot::Bno055::resetAfterConfigSettles(uint32_t now_ms)
{
    if (phaseElapsedMs(now_ms) < CONFIG_SETTLE_MS)
        return;
    requestReset();
    startTimedPhase(WAITING_RESET, now_ms);
    next_poll_elapsed_ms_ = RESET_FIRST_POLL_MS;
}

void diffbot::Bno055::pollResetCompletion(uint32_t now_ms)
{
    const uint32_t elapsed_ms = phaseElapsedMs(now_ms);
    // The deadline comes first: from it on, no poll counts, even from a call that comes late
    if (elapsed_ms >= RESET_TIMEOUT_MS)
    {
        transitionTo(FAILED);
        return;
    }
    if (elapsed_ms < next_poll_elapsed_ms_)
        return;
    if (chipAnswers())
        configureAndRequestImuMode(now_ms);
    else
        next_poll_elapsed_ms_ = elapsed_ms + RESET_POLL_MS;
}

void diffbot::Bno055::waitForImuMode(uint32_t now_ms)
{
    const uint32_t elapsed_ms = phaseElapsedMs(now_ms);
    // The fusion's deadline counts from the IMU mode request, so it holds here already: a loop
    // that comes back only after it fails the start-up now, not at the next call
    if (elapsed_ms >= FUSION_TIMEOUT_MS)
    {
        transitionTo(FAILED);
        return;
    }
    if (elapsed_ms < IMU_SETTLE_MS)
        return;
    // Poll SYS_STATUS from the next call on. The timed phase goes on: the fusion's deadline
    // counts from the IMU mode request
    transitionTo(WAITING_FUSION);
    next_poll_elapsed_ms_ = elapsed_ms;
}

void diffbot::Bno055::pollFusionStatus(uint32_t now_ms)
{
    // The fusion starts after the mode switch. Until then SYS_STATUS reports an initialization
    // state. Wait for it, but not past the deadline, which comes first as in WAITING_RESET
    const uint32_t elapsed_ms = phaseElapsedMs(now_ms);
    if (elapsed_ms >= FUSION_TIMEOUT_MS)
    {
        transitionTo(FAILED);
        return;
    }
    if (elapsed_ms < next_poll_elapsed_ms_)
        return;
    uint8_t sys_status = 0;
    const bool read_ok = readRegister(REG_SYS_STATUS, sys_status);
    if (read_ok && sys_status == SYS_STATUS_FUSION_RUNNING)
        transitionTo(READY);
    else if (read_ok && sys_status == SYS_STATUS_ERROR)
        transitionTo(FAILED);
    else
        next_poll_elapsed_ms_ = elapsed_ms + FUSION_POLL_MS;
}

void diffbot::Bno055::transitionTo(State next)
{
    state_ = next;
}

void diffbot::Bno055::startTimedPhase(State next, uint32_t now_ms)
{
    transitionTo(next);
    phase_started_ms_ = now_ms;
    next_poll_elapsed_ms_ = 0;
}

uint32_t diffbot::Bno055::phaseElapsedMs(uint32_t now_ms) const
{
    // Unsigned, so it stays right when millis() wraps
    return now_ms - phase_started_ms_;
}

bool diffbot::Bno055::chipAnswers()
{
    uint8_t id = 0;
    return readRegister(REG_CHIP_ID, id) && id == CHIP_ID;
}

void diffbot::Bno055::requestReset()
{
    // A reset gives defined settings after a firmware restart, when the sensor kept running.
    // The chip may reset before it acknowledges the write, so its result says nothing: the
    // polls of the chip ID afterwards tell whether the chip came back
    (void)bus_.writeRegister(address_, REG_SYS_TRIGGER, SYS_TRIGGER_RESET);
}

void diffbot::Bno055::configureAndRequestImuMode(uint32_t now_ms)
{
    if (!configure() || !bus_.writeRegister(address_, REG_OPR_MODE, MODE_IMU))
    {
        transitionTo(FAILED);
        return;
    }
    // The timed phase that starts here also covers WAITING_FUSION
    startTimedPhase(WAITING_IMU_MODE, now_ms);
}

bool diffbot::Bno055::configure()
{
    if (!selectPageZero())
        return false;
    if (!writeSettings())
        return false;
    if (!selectInternalClock())
        return false;
    return verifySettings() && verifyInternalClock();
}

bool diffbot::Bno055::selectPageZero()
{
    // The registers that the driver uses are on page 0
    if (!bus_.writeRegister(address_, REG_PAGE_ID, PAGE_ZERO))
        return false;
    uint8_t page = 0;
    return readRegister(REG_PAGE_ID, page) && page == PAGE_ZERO;
}

bool diffbot::Bno055::writeSettings()
{
    for (const RegisterSetting& setting : SETTINGS)
    {
        if (!bus_.writeRegister(address_, setting.reg, setting.value))
            return false;
    }
    return true;
}

bool diffbot::Bno055::selectInternalClock()
{
    return bus_.writeRegister(address_, REG_SYS_TRIGGER, SYS_TRIGGER_INTERNAL_CLOCK);
}

bool diffbot::Bno055::verifySettings()
{
    for (const RegisterSetting& setting : SETTINGS)
    {
        uint8_t value = 0;
        if (!readRegister(setting.reg, value) || value != setting.value)
            return false;
    }
    return true;
}

bool diffbot::Bno055::verifyInternalClock()
{
    // SYS_TRIGGER also holds command bits, which aren't settings: only CLK_SEL counts
    uint8_t trigger = 0;
    return readRegister(REG_SYS_TRIGGER, trigger) && (trigger & SYS_TRIGGER_CLK_SEL) == 0;
}

bool diffbot::Bno055::readRegister(uint8_t reg, uint8_t& value)
{
    return bus_.readRegisters(address_, reg, &value, 1);
}

bool diffbot::Bno055::read(ImuSample& sample)
{
    if (state_ != READY)
        return false;
    // One 18-byte burst read from ACC_DATA_X_LSB to GYR_DATA_Z_MSB, including the unused
    // magnetometer bytes (datasheet sections 3.7 and 4.6)
    uint8_t data[DATA_LENGTH];
    if (!bus_.readRegisters(address_, REG_ACC_DATA_X_LSB, data, DATA_LENGTH))
        return false;
    uint8_t status[STATUS_LENGTH];
    if (!bus_.readRegisters(address_, REG_CALIB_STAT, status, STATUS_LENGTH))
        return false;
    uint8_t mode = 0;
    if (!readRegister(REG_OPR_MODE, mode))
        return false;
    // status[] starts at CALIB_STAT (0x35)
    const uint8_t sys_status = status[REG_SYS_STATUS - REG_CALIB_STAT];
    const uint8_t units = status[REG_UNIT_SEL - REG_CALIB_STAT];
    if (mode != MODE_IMU || units != UNITS || sys_status == SYS_STATUS_ERROR)
    {
        // The sensor reset itself or reports a system error: start over
        restart();
        return false;
    }
    if (sys_status != SYS_STATUS_FUSION_RUNNING)
        return false;
    // Only after all the checks, so that a failed read leaves the caller's sample as it was
    bno055Convert(data, sample);
    sample.calibration = status[0];
    return true;
}
