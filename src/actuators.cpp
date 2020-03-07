// Module to handle actuator input/output and mixing.

#include "imu.h"
#include "pwm.h"
#include "setup_pwm.h"
#include "sbus.h"

#include "actuators.h"

// official flight command values.  These could source from the RC
// receiver or the autopilot depending on the auto/manual selection
// switch state.  These are pre-mix commands and will be mixed and
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

// reset pwm output rates to safe startup defaults
void actuators_t::pwm_defaults() {
    for ( int i = 0; i < message::pwm_channels; i++ ) {
         config.pwm_hz[i] = 50;    
    }
}

// reset actuator gains (reversing) to startup defaults
void actuators_t::act_gain_defaults() {
    for ( int i = 0; i < message::pwm_channels; i++ ) {
        config.act_gain[i] = 1.0;
    }
}

// reset sas parameters to startup defaults
void actuators_t::sas_defaults() {
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
void actuators_t::mixing_defaults() {
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
void actuators_t::sas_update( float control_norm[SBUS_CHANNELS] ) {
    // mixing modes that work at the 'command' level (before actuator
    // value assignment)

    float tune = 1.0;
    if ( config.sas_tune ) {
        tune = config.sas_max_gain * sbus.receiver_norm[7];
        if ( tune < 0.0 ) {
            tune = 0.0;
        } else if ( tune > 2.0 ) {
            tune = 2.0;
        }
    }

    if ( config.sas_rollaxis ) {
        control_norm[3] -= tune * config.sas_rollgain * imu.p;
    }
    if ( config.sas_pitchaxis ) {
        control_norm[4] += tune * config.sas_pitchgain * imu.q;
    }
    if ( config.sas_yawaxis ) {
        control_norm[5] += tune * config.sas_yawgain * imu.r;
    }
}

// compute the actuator (servo) values for each channel.  Handle all
// the requested mixing modes here.
void actuators_t::mixing_update( float control_norm[SBUS_CHANNELS] ) {
    bool throttle_enabled = control_norm[1] > 0.0;
    aileron_cmd = control_norm[3];
    elevator_cmd = control_norm[4];
    throttle_cmd = control_norm[2];
    rudder_cmd = control_norm[5];
    flap_cmd = control_norm[6];
    gear_cmd = control_norm[7];
        
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

    if ( throttle_enabled ) {
        actuator_norm[0] = throttle_cmd;
    } else {
        actuator_norm[0] = 0.0;
    }
    actuator_norm[1] = aileron_cmd;
    actuator_norm[2] = elevator_cmd;
    actuator_norm[3] = rudder_cmd;
    actuator_norm[4] = flap_cmd;
    actuator_norm[5] = gear_cmd;

    // elevon and flaperon mixing are mutually exclusive
    if ( config.mix_elevon ) {
        actuator_norm[1] = config.mix_Gea * aileron_cmd + config.mix_Gee * elevator_cmd;
        actuator_norm[2] = config.mix_Gea * aileron_cmd - config.mix_Gee * elevator_cmd;
    } else if ( config.mix_flaperon ) {
        actuator_norm[1] = config.mix_Gfa * aileron_cmd + config.mix_Gff * flap_cmd;
        actuator_norm[4] = -config.mix_Gfa * aileron_cmd + config.mix_Gff * flap_cmd;
    }
    // vtail mixing can't work with elevon mixing
    if ( config.mix_vtail && !config.mix_elevon) {
        actuator_norm[2] = config.mix_Gve * elevator_cmd + config.mix_Gvr * rudder_cmd;
        actuator_norm[3] = config.mix_Gve * elevator_cmd - config.mix_Gvr * rudder_cmd;
    }
    if ( config.mix_diff_thrust ) {
        // fixme: never tested in the wild (need to think through channel assignments)
        // actuator_norm[0] = config.mix_Gtt * throttle_cmd + config.mix_Gtr * rudder_cmd;
        // actuator_norm[5] = config.mix_Gtt * throttle_cmd - config.mix_Gtr * rudder_cmd;
    }

    // compute pwm actuator output values from the normalized values
    pwm.norm2pwm( actuator_norm, pwm.actuator_pwm );
}

// set (zero) default raw actuator values
void actuators_t::setup() {
    for ( int i = 0; i < SBUS_CHANNELS; i++ ) {
        actuator_norm[i] = 0.0;
    }
    pwm.norm2pwm(actuator_norm, pwm.actuator_pwm);
}

// global shared instance
actuators_t actuators;
