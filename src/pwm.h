#pragma once

#include <Arduino.h>

// max number of pwm output channels
const int PWM_CHANNELS = 8;

class pwm_t {
private:
    uint16_t gen_pwm_test_value();
    
public:
    uint16_t actuator_pwm[PWM_CHANNELS];
    void setup(int board);
    void update(uint8_t test_pwm_channel = -1);
    void norm2pwm( float *norm, uint16_t *pwm);
};

// a global instance is available for use
extern pwm_t pwm;
