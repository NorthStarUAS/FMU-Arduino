// Module to handle actuator input/output and mixing.

#include "imu.h"
#include "pilot.h"

#include "mixer.h"

void mixer_t::setup() {
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        outputs[i] = 0.0;
    }
}

// reset sas parameters to startup defaults
void mixer_t::sas_defaults() {
    config.sas_rollaxis = false;
    config.sas_pitchaxis = false;
    config.sas_yawaxis = false;
    config.sas_tune = false;

    config.sas_rollgain = 0.0;
    config.sas_pitchgain = 0.0;
    config.sas_yawgain = 0.0;
    config.sas_max_gain = 2.0;
};


// reset mixing parameters to startup defaults
void mixer_t::mixing_defaults() {
    config.mix_autocoord = false;
    config.mix_throttle_trim = false;
    config.mix_flap_trim = false;
    config.mix_elevon = false;
    config.mix_flaperon = false;
    config.mix_vtail = false;
    config.mix_diff_thrust = false;

    config.mix_Gac = 0.5;       // aileron gain for autocoordination
    config.mix_Get = -0.1;      // elevator trim w/ throttle gain
    config.mix_Gef = 0.1;       // elevator trim w/ flap gain

    config.mix_Gea = 1.0;       // aileron gain for elevons
    config.mix_Gee = 1.0;       // elevator gain for elevons
    config.mix_Gfa = 1.0;       // aileron gain for flaperons
    config.mix_Gff = 1.0;       // flaps gain for flaperons
    config.mix_Gve = 1.0;       // elevator gain for vtail
    config.mix_Gvr = 1.0;       // rudder gain for vtail
    config.mix_Gtt = 1.0;       // throttle gain for diff thrust
    config.mix_Gtr = 0.1;       // rudder gain for diff thrust
};


// compute the sas compensation in normalized 'command' space so that
// we can do proper output channel mixing later
void mixer_t::sas_update() {
    // mixing modes that work at the 'command' level (before actuator
    // value assignment)

    float tune = 1.0;
    if ( config.sas_tune ) {
        tune = config.sas_max_gain * pilot.manual_inputs[7];
        if ( tune < 0.0 ) {
            tune = 0.0;
        } else if ( tune > 2.0 ) {
            tune = 2.0;
        }
    }

    if ( config.sas_rollaxis ) {
        aileron_cmd -= tune * config.sas_rollgain * imu.get_p();
    }
    if ( config.sas_pitchaxis ) {
        elevator_cmd += tune * config.sas_pitchgain * imu.get_q();
    }
    if ( config.sas_yawaxis ) {
        rudder_cmd += tune * config.sas_yawgain * imu.get_r();
    }
}

// compute the actuator (servo) values for each channel.  Handle all
// the requested mixing modes here.
void mixer_t::mixing_update() {
    // mixing modes that work at the 'command' level (before actuator
    // value assignment)
    if ( config.mix_autocoord ) {
        rudder_cmd += config.mix_Gac * aileron_cmd;
    }
    if ( config.mix_throttle_trim ) {
        elevator_cmd += config.mix_Get * throttle_cmd;
    }
    if ( config.mix_flap_trim ) {
        elevator_cmd += config.mix_Gef * flap_cmd;
    }

    if ( pilot.throttle_safety() ) {
        outputs[0] = 0.0;
    } else {
        outputs[0] = throttle_cmd;
    }
    outputs[1] = aileron_cmd;
    outputs[2] = elevator_cmd;
    outputs[3] = rudder_cmd;
    outputs[4] = flap_cmd;
    outputs[5] = gear_cmd;

    // elevon and flaperon mixing are mutually exclusive
    if ( config.mix_elevon ) {
        outputs[1] = config.mix_Gea * aileron_cmd + config.mix_Gee * elevator_cmd;
        outputs[2] = config.mix_Gea * aileron_cmd - config.mix_Gee * elevator_cmd;
    } else if ( config.mix_flaperon ) {
        outputs[1] = config.mix_Gfa * aileron_cmd + config.mix_Gff * flap_cmd;
        outputs[4] = -config.mix_Gfa * aileron_cmd + config.mix_Gff * flap_cmd;
    }
    // vtail mixing can't work with elevon mixing
    if ( config.mix_vtail && !config.mix_elevon) {
        outputs[2] = config.mix_Gve * elevator_cmd + config.mix_Gvr * rudder_cmd;
        outputs[3] = config.mix_Gve * elevator_cmd - config.mix_Gvr * rudder_cmd;
    }
    if ( config.mix_diff_thrust ) {
        // fixme: never tested in the wild (need to think through channel assignments)
        // outputs[0] = config.mix_Gtt * throttle_cmd + config.mix_Gtr * rudder_cmd;
        // outputs[5] = config.mix_Gtt * throttle_cmd - config.mix_Gtr * rudder_cmd;
    }
}

void mixer_t::update( float control_norm[SBUS_CHANNELS] ) {
    // initialize commands
    aileron_cmd = pilot.get_aileron();
    elevator_cmd = pilot.get_elevator();
    throttle_cmd = pilot.get_throttle();
    rudder_cmd = pilot.get_rudder();
    flap_cmd = pilot.get_flap();
    gear_cmd = pilot.get_gear();
    
    sas_update();
    mixing_update();

    // compute pwm actuator output values from the normalized values
    pwm.norm2pwm( outputs );
}

// global shared instance
mixer_t mixer;
