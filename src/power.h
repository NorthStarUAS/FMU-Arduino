#pragma once

class power_t {
private:
    const float analogResolution = 65535.0f;
    const float pwr_scale = 11.0f;
    const float avionics_scale = 2.0f;
    uint8_t avionics_pin;
    uint8_t source_volt_pin;
    uint8_t atto_volts_pin = A2;
    uint8_t atto_amps_pin = A3;
    
public:
    float pwr1_v = 0.0;
    float pwr2_v = 0.0;
    float avionics_v = 0.0;
    float pwr_a = 0.0;
    void setup();
    void update();
};

extern power_t power;
