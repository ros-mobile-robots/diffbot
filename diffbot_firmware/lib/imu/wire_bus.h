#ifndef DIFFBOT_WIRE_BUS_H
#define DIFFBOT_WIRE_BUS_H

#include "i2c_bus.h"

namespace diffbot
{
    /**
     * @brief \ref I2cBus on the Arduino Wire library, for the Teensy.
     *
     * A template on the Wire class (TwoWire on the Teensy), so that the native unit tests run it
     * with a fake one, without the Arduino core (test/test_wire_bus).
     */
    template <class Wire>
    class WireBusT : public I2cBus
    {
    public:
        explicit WireBusT(Wire& wire) : wire_(wire) {}

        bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) override
        {
            wire_.beginTransmission(address);
            wire_.write(reg);
            wire_.write(value);
            return wire_.endTransmission() == 0;
        }

        bool readRegisters(uint8_t address, uint8_t reg, uint8_t* data, size_t length) override
        {
            wire_.beginTransmission(address);
            wire_.write(reg);
            // A repeated start: the register pointer stays set for the read
            if (wire_.endTransmission(false) != 0)
                return false;
            // Check both the count that requestFrom reports and the bytes really received: on a
            // failed bus acquisition, the Teensy 4's Wire can report a count without data
            if (wire_.requestFrom(address, (uint8_t)length) != length || wire_.available() != (int)length)
            {
                while (wire_.available())
                    wire_.read();
                return false;
            }
            for (size_t i = 0; i < length; ++i)
                data[i] = wire_.read();
            return true;
        }

    private:
        Wire& wire_;
    };
}

#endif // DIFFBOT_WIRE_BUS_H
