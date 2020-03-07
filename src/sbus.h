#pragma once

#include "setup_sbus.h"

class sbus_t {
public:
    float receiver_norm[SBUS_CHANNELS];
    uint8_t receiver_flags = 0x00;
    void setup();
    void parse();
    bool process();
    void raw2norm( uint16_t *raw, float *norm );
};

extern sbus_t sbus;
