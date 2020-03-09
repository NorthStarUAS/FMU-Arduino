#pragma once

#include "aura4_messages.h"
#include "pwm.h"
#include "sbus.h"

class actuators_t {
private:
    // Command input values.  These could source from the RC receiver
    // or the autopilot depending on the auto/manual selection switch
    // state.  These are pre-mix commands and will be mixed and
    // written to the actuators for both manual and autonomous flight
    // modes.
    float aileron_cmd = 0.0;
    float elevator_cmd = 0.0;
    float throttle_cmd = 0.0;
    float rudder_cmd = 0.0;
    float gear_cmd = 0.0;
    float flap_cmd = 0.0;
    float ch7_cmd = 0.0;
    float ch8_cmd = 0.0;
    
    float actuator_norm[PWM_CHANNELS];
    void sas_update();
    void mixing_update();

public:
    message::config_actuators_t config;
    void setup();
    void pwm_defaults();
    void act_gain_defaults();
    void sas_defaults();
    void mixing_defaults();
    void update( float control_norm[SBUS_CHANNELS] );
};

// a global instance is available for use
extern actuators_t actuators;
