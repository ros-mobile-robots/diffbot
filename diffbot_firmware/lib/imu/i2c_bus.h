#ifndef DIFFBOT_I2C_BUS_H
#define DIFFBOT_I2C_BUS_H

#include <stddef.h>
#include <stdint.h>

namespace diffbot
{
    /**
     * @brief The few I2C transfers an IMU driver needs, so that the driver doesn't depend on the
     * Arduino Wire library: on the Teensy, \ref WireBus implements it; the native unit tests use a
     * fake bus. Every transfer reports whether it succeeded, so that a driver can tell a failed
     * read from valid data.
     */
    class I2cBus
    {
    public:
        virtual ~I2cBus() {}

        /**
         * @brief Writes one byte to a register of the device at \p address.
         * @return true if the device acknowledged the address, the register and the value.
         */
        virtual bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) = 0;

        /**
         * @brief Reads \p length bytes starting at register \p reg of the device at \p address.
         * @return true if all \p length bytes arrived.
         */
        virtual bool readRegisters(uint8_t address, uint8_t reg, uint8_t* data, size_t length) = 0;
    };
}

#endif // DIFFBOT_I2C_BUS_H
