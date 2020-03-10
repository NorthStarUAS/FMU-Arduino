// Module to handle actuator input/output and mixing.

#include "mixer.h"
#include "actuators.h"

// reset pwm output rates to safe startup defaults
void actuators_t::pwm_defaults() {
    // for ( int i = 0; i < message::pwm_channels; i++ ) {
    //      config.pwm_hz[i] = 50;    
    // }
}

// reset actuator gains (reversing) to startup defaults
void actuators_t::act_gain_defaults() {
    // for ( int i = 0; i < message::pwm_channels; i++ ) {
    //     config.act_gain[i] = 1.0;
    // }
}

// set (zero) default raw actuator values
// FIXME: doesn't make sense to depend on an external module being initialized first I don't think.
void actuators_t::setup() {
    pwm.norm2pwm( mixer.outputs.data() );
}

void actuators_t::update() {
    // compute pwm actuator output values from the normalized values
    pwm.norm2pwm( mixer.outputs.data() );
}

// global shared instance
actuators_t actuators;
