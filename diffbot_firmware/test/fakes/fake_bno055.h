// A BNO055 on a fake I2C bus, for the native unit tests: a register array that can fail
// transfers, ignore writes to one register, and stay silent after a reset until the test says
// it's back.
#ifndef DIFFBOT_FAKE_BNO055_H
#define DIFFBOT_FAKE_BNO055_H

#include <string.h>

#include "bno055.h"

class FakeBno055Bus : public diffbot::I2cBus
{
public:
    typedef diffbot::Bno055 Bno055;

    uint8_t registers[128];
    uint8_t address = Bno055::ADDRESS_DEFAULT;
    bool fail_reads = false;
    bool fail_writes = false;
    int ignore_writes_to = -1;
    bool resetting = false;
    bool back_after_reset = true;
    int reset_count = 0;
    /// Every writeRegister() and readRegisters() call, successful or not
    int transfers = 0;
    /// SYS_STATUS after the switch to IMUPLUS: the fusion runs
    uint8_t fusion_status = Bno055::SYS_STATUS_FUSION_RUNNING;
    /// Registers read in one transfer must not include this reserved one
    int reserved_reads = 0;
    /// Data reads (from ACC_DATA_X_LSB), and a function that each one calls, for example to
    /// advance a fake clock
    int data_reads = 0;
    void (*on_data_read)() = nullptr;

    FakeBno055Bus()
    {
        memset(registers, 0, sizeof(registers));
        registers[Bno055::REG_CHIP_ID] = Bno055::CHIP_ID;
        registers[Bno055::REG_UNIT_SEL] = 0x80; // the reset value
        registers[Bno055::REG_AXIS_MAP_CONFIG] = Bno055::AXIS_MAP_CONFIG_DEFAULT;
    }

    bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) override
    {
        ++transfers;
        if (fail_writes || addr != address || (resetting && !back_after_reset))
            return false;
        if (reg == Bno055::REG_SYS_TRIGGER && (value & 0x20))
        {
            ++reset_count;
            resetting = true;
            registers[Bno055::REG_OPR_MODE] = Bno055::MODE_CONFIG;
            registers[Bno055::REG_UNIT_SEL] = 0x80;
            registers[Bno055::REG_SYS_STATUS] = 0x00; // idle
            return false; // the chip doesn't acknowledge during its reset
        }
        if (reg == ignore_writes_to)
            return true;
        registers[reg] = value;
        if (reg == Bno055::REG_OPR_MODE)
            registers[Bno055::REG_SYS_STATUS] = value == Bno055::MODE_IMUPLUS ? fusion_status : 0x00;
        return true;
    }

    bool readRegisters(uint8_t addr, uint8_t reg, uint8_t* data, size_t length) override
    {
        ++transfers;
        if (fail_reads || addr != address || (resetting && !back_after_reset))
            return false;
        if (reg <= 0x3C && reg + length > 0x3C)
            ++reserved_reads;
        if (reg == Bno055::REG_ACC_DATA_X_LSB)
        {
            ++data_reads;
            if (on_data_read)
                on_data_read();
        }
        memcpy(data, &registers[reg], length);
        return true;
    }
};

inline void setLe16(uint8_t* bytes, int16_t value)
{
    bytes[0] = (uint8_t)(value & 0xFF);
    bytes[1] = (uint8_t)((uint16_t)value >> 8);
}

#endif // DIFFBOT_FAKE_BNO055_H
