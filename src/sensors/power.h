#pragma once

#include "../props2.h"
#include "../util/lowpass.h"

class power_t {

private:

    PropertyNode config_power_node;

    uint8_t cells = 1;

    uint8_t avionics_pin;
    uint8_t source_volt_pin;
    uint8_t atto_volts_pin = A2;
    uint8_t atto_amps_pin = A3;
    float battery_cal = 1.0;

public:

    LowPassFilter battery_volts;
    LowPassFilter avionics_volts;
    float battery_amp = 0.0;

    void init();
    void update(float dt);
};