#include <Encoder.h>


namespace diffbot 
{
    class Encoder
    {
    public:
        ::Encoder encoder;

        Encoder(uint8_t pin1, uint8_t pin2);

        int getRPM();

        inline int32_t read() { return encoder.read(); };
        inline void write(int32_t p) { encoder.write(p); };

    private:
        int counts_per_rev_;
        unsigned long prev_update_time_;
        long prev_encoder_ticks_;
    };
}