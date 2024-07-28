#pragma once

#include <Arduino.h>

const int SBUS_CHANNELS = 16;

// define if an sbus input channel is symmetrical or not (i.e. mapped to
// [0,1] for throttle, flaps, spoilers; [-1,1] for aileron, elevator, rudder
static const bool sbus_symmetrical[SBUS_CHANNELS] = {1, 1, 0, 1, 1, 1, 1, 0, 0};

class sbus_t {

private:

    static const uint8_t SBUS_PAYLOAD_LEN = 23;
    static const uint8_t SBUS_FRAMELOST = (1 << 2);
    static const uint8_t SBUS_FAILSAFE = (1 << 3);

    typedef union {
        uint8_t buf[SBUS_PAYLOAD_LEN];

        // Structure defining the contents of the SBUS data payload
        // (23 bytes).  Each of the channel fields (ch1:ch16) occupies
        // 11 bytes.
        struct __attribute__ ((packed)) {
            uint32_t ch1          : 11;
            uint32_t ch2          : 11;
            uint32_t ch3_lo       : 10;

            uint32_t ch3_hi       :  1;
            uint32_t ch4          : 11;
            uint32_t ch5          : 11;
            uint32_t ch6_lo       :  9;

            uint32_t ch6_hi       :  2;
            uint32_t ch7          : 11;
            uint32_t ch8          : 11;
            uint32_t ch9_lo       :  8;

            uint32_t ch9_hi       :  3;
            uint32_t ch10         : 11;
            uint32_t ch11         : 11;
            uint32_t ch12_lo      :  7;

            uint32_t ch12_hi      :  4;
            uint32_t ch13         : 11;
            uint32_t ch14         : 11;
            uint32_t ch15_lo      :  6;

            uint32_t ch15_hi      :  5;
            uint32_t ch16         : 11;
            uint32_t ch17         :  1; // digital channel
            uint32_t ch18         :  1; // digital channel
            uint32_t frame_lost   :  1;
            uint32_t failsafe_act :  1;
        };
    } SBUS_DATA_U;
    SBUS_DATA_U sbus_data;
    uint16_t raw_val[SBUS_CHANNELS];

public:

    uint8_t receiver_flags = 0x00;
    void init();
    void parse();
    bool process();

    float norm_val[SBUS_CHANNELS];    // normalized value (range of -1 to 1, or 0 to 1)
    uint16_t pwm_val[SBUS_CHANNELS];  // equivalent-ish PWM value (range of 1000-2000)
};
