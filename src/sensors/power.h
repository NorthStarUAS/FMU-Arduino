#pragma once

#include "../props2.h"

class power_t {

private:

    PropertyNode config_power_node;

    uint8_t cells = 1;

    const float analogResolution = 65535.0f;
    const float pwr_scale = 11.0f;
    const float avionics_scale = 2.0f;
    uint8_t avionics_pin;
    uint8_t source_volt_pin;
    uint8_t atto_volts_pin = A2;
    uint8_t atto_amps_pin = A3;

public:

    float avionics_volt = 0.0;
    float battery_volt = 0.0;
    float battery_amp = 0.0;

    void init();
    void update();
};