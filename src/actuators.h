#pragma once

#include "aura4_messages.h"
#include "pwm.h"
#include "sbus.h"

class actuators_t {
private:
    float actuator_norm[PWM_CHANNELS];

public:
    message::config_actuators_t config;
    void setup();
    void pwm_defaults();
    void act_gain_defaults();
    void sas_defaults();
    void mixing_defaults();
    void sas_update( float control_norm[SBUS_CHANNELS] );
    void mixing_update( float control_norm[SBUS_CHANNELS] );
};

// a global instance is available for use
extern actuators_t actuators;
