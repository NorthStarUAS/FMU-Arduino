#pragma once

#include "aura4_messages.h"
#include "pwm.h"
#include "sensors/sbus/sbus.h"

class actuators_t {

public:
    message::config_actuators_t config;
    void setup();
    void pwm_defaults();
    void act_gain_defaults();
    void update();
};

extern actuators_t actuators;
