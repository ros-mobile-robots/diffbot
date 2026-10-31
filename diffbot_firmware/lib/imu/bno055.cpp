#include "bno055.h"

namespace
{
    int16_t le16(const uint8_t* bytes)
    {
        return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
    }
}

void diffbot::bno055Convert(const uint8_t data[18], ImuSample& sample)
{
    // Bytes 0-5: accelerometer x, y, z; 6-11: magnetometer (unused); 12-17: gyroscope x, y, z
    for (int i = 0; i < 3; ++i)
    {
        sample.linear_acceleration[i] = le16(&data[2 * i]) / 100.0f;
        sample.angular_velocity[i] = le16(&data[12 + 2 * i]) / 900.0f;
    }
}

diffbot::Bno055::Bno055(I2cBus& bus, uint8_t address)
    : bus_(bus), address_(address), state_(STARTING), step_started_ms_(0), next_step_ms_(0)
{
}

void diffbot::Bno055::restart()
{
    state_ = STARTING;
}

void diffbot::Bno055::enter(State state, uint32_t now_ms)
{
    state_ = state;
    step_started_ms_ = now_ms;
}

diffbot::Bno055::State diffbot::Bno055::update(uint32_t now_ms)
{
    // Time since the current state began; unsigned, so it stays right when millis() wraps
    uint32_t elapsed = now_ms - step_started_ms_;
    uint8_t id = 0;

    switch (state_)
    {
    case STARTING:
        if (!bus_.readRegisters(address_, REG_CHIP_ID, &id, 1) || id != CHIP_ID)
        {
            state_ = ABSENT;
            break;
        }
        // Settings can only change in configuration mode
        if (!bus_.writeRegister(address_, REG_OPR_MODE, MODE_CONFIG))
        {
            state_ = FAILED;
            break;
        }
        enter(WAITING_CONFIG, now_ms);
        break;

    case WAITING_CONFIG:
        if (elapsed < CONFIG_MODE_MS)
            break;
        // A reset gives defined settings after a firmware restart, when the sensor kept running.
        // The chip doesn't answer during the reset, so the write may not be acknowledged.
        bus_.writeRegister(address_, REG_SYS_TRIGGER, 0x20);
        enter(WAITING_RESET, now_ms);
        next_step_ms_ = RESET_FIRST_POLL_MS;
        break;

    case WAITING_RESET:
        if (elapsed < next_step_ms_)
            break;
        if (bus_.readRegisters(address_, REG_CHIP_ID, &id, 1) && id == CHIP_ID)
        {
            if (!configure() || !bus_.writeRegister(address_, REG_OPR_MODE, MODE_IMUPLUS))
            {
                state_ = FAILED;
                break;
            }
            enter(WAITING_MODE, now_ms);
        }
        else if (elapsed >= RESET_TIMEOUT_MS)
            state_ = FAILED;
        else
            next_step_ms_ = elapsed + RESET_POLL_MS;
        break;

    case WAITING_MODE:
        if (elapsed < IMUPLUS_MODE_MS)
            break;
        // Poll SYS_STATUS from the next call on; the time still counts from the mode switch
        state_ = WAITING_FUSION;
        next_step_ms_ = elapsed;
        break;

    case WAITING_FUSION:
    {
        if (elapsed < next_step_ms_)
            break;
        // The fusion starts after the mode switch; until then SYS_STATUS reports an
        // initialization state. Wait for it, but not forever
        uint8_t sys_status = 0;
        bool ok = bus_.readRegisters(address_, REG_SYS_STATUS, &sys_status, 1);
        if (ok && sys_status == SYS_STATUS_FUSION_RUNNING)
            state_ = READY;
        else if ((ok && sys_status == SYS_STATUS_ERROR) || elapsed >= FUSION_TIMEOUT_MS)
            state_ = FAILED;
        else
            next_step_ms_ = elapsed + FUSION_POLL_MS;
        break;
    }

    case READY:
    case ABSENT:
    case FAILED:
        break;
    }
    return state_;
}

bool diffbot::Bno055::configure()
{
    // Write each setting explicitly instead of relying on the reset, and read them back
    const uint8_t settings[][2] = {
        {REG_PAGE_ID, 0x00},
        {REG_PWR_MODE, 0x00}, // normal power mode
        {REG_UNIT_SEL, UNITS},
        {REG_AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_DEFAULT},
        {REG_AXIS_MAP_SIGN, AXIS_MAP_SIGN_DEFAULT},
        {REG_SYS_TRIGGER, 0x00}, // internal oscillator, no reset
    };
    for (const auto& setting : settings)
    {
        if (!bus_.writeRegister(address_, setting[0], setting[1]))
            return false;
    }
    for (int i = 1; i < 5; ++i)
    {
        uint8_t value = 0;
        if (!bus_.readRegisters(address_, settings[i][0], &value, 1) || value != settings[i][1])
            return false;
    }
    return true;
}

bool diffbot::Bno055::read(ImuSample& sample)
{
    if (state_ != READY)
        return false;
    uint8_t data[DATA_LENGTH];
    uint8_t status[STATUS_LENGTH];
    uint8_t mode = 0;
    if (!bus_.readRegisters(address_, REG_ACC_DATA_X_LSB, data, DATA_LENGTH)
        || !bus_.readRegisters(address_, REG_CALIB_STAT, status, STATUS_LENGTH)
        || !bus_.readRegisters(address_, REG_OPR_MODE, &mode, 1))
        return false;
    // status[] starts at CALIB_STAT (0x35)
    uint8_t sys_status = status[REG_SYS_STATUS - REG_CALIB_STAT];
    uint8_t units = status[REG_UNIT_SEL - REG_CALIB_STAT];
    if (mode != MODE_IMUPLUS || units != UNITS || sys_status == SYS_STATUS_ERROR)
    {
        // The sensor reset itself or reports a system error: start over
        restart();
        return false;
    }
    if (sys_status != SYS_STATUS_FUSION_RUNNING)
        return false;
    bno055Convert(data, sample);
    sample.calibration = status[0];
    return true;
}
