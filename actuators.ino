// Module to handle actuator input/output and mixing.

#include "setup_pwm.h"

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
void pwm_defaults() {
    for ( int i = 0; i < message_pwm_channels; i++ ) {
         config_actuators.pwm_hz[i] = 50;    
    }
}

// reset actuator gains (reversing) to startup defaults
void act_gain_defaults() {
    for ( int i = 0; i < message_pwm_channels; i++ ) {
        config_actuators.act_gain[i] = 1.0;
    }
}

// reset sas parameters to startup defaults
void sas_defaults() {
    config_actuators.sas_rollaxis = false;
    config_actuators.sas_pitchaxis = false;
    config_actuators.sas_yawaxis = false;
    config_actuators.sas_tune = false;

    config_actuators.sas_rollgain = 0.0;
    config_actuators.sas_pitchgain = 0.0;
    config_actuators.sas_yawgain = 0.0;
    config_actuators.sas_max_gain = 2.0;
};


// reset mixing parameters to startup defaults
void mixing_defaults() {
    config_actuators.mix_autocoord = false;
    config_actuators.mix_throttle_trim = false;
    config_actuators.mix_flap_trim = false;
    config_actuators.mix_elevon = false;
    config_actuators.mix_flaperon = false;
    config_actuators.mix_vtail = false;
    config_actuators.mix_diff_thrust = false;

    config_actuators.mix_Gac = 0.5;       // aileron gain for autocoordination
    config_actuators.mix_Get = -0.1;      // elevator trim w/ throttle gain
    config_actuators.mix_Gef = 0.1;       // elevator trim w/ flap gain

    config_actuators.mix_Gea = 1.0;       // aileron gain for elevons
    config_actuators.mix_Gee = 1.0;       // elevator gain for elevons
    config_actuators.mix_Gfa = 1.0;       // aileron gain for flaperons
    config_actuators.mix_Gff = 1.0;       // flaps gain for flaperons
    config_actuators.mix_Gve = 1.0;       // elevator gain for vtail
    config_actuators.mix_Gvr = 1.0;       // rudder gain for vtail
    config_actuators.mix_Gtt = 1.0;       // throttle gain for diff thrust
    config_actuators.mix_Gtr = 0.1;       // rudder gain for diff thrust
};


// compute the sas compensation in normalized 'command' space so that
// we can do proper output channel mixing later
void sas_update( float control_norm[SBUS_CHANNELS] ) {
    // mixing modes that work at the 'command' level (before actuator
    // value assignment)

    float tune = 1.0;
    if ( config_actuators.sas_tune ) {
        tune = config_actuators.sas_max_gain * receiver_norm[7];
        if ( tune < 0.0 ) {
            tune = 0.0;
        } else if ( tune > 2.0 ) {
            tune = 2.0;
        }
    }

    if ( config_actuators.sas_rollaxis ) {
        control_norm[3] -= tune * config_actuators.sas_rollgain * imu_calib[3];  // p
    }
    if ( config_actuators.sas_pitchaxis ) {
        control_norm[4] += tune * config_actuators.sas_pitchgain * imu_calib[4]; // q
    }
    if ( config_actuators.sas_yawaxis ) {
        control_norm[5] += tune * config_actuators.sas_yawgain * imu_calib[5];   // r
    }
}

// compute the actuator (servo) values for each channel.  Handle all
// the requested mixing modes here.
void mixing_update( float control_norm[SBUS_CHANNELS] ) {
    bool throttle_enabled = control_norm[1] > 0.0;
    aileron_cmd = control_norm[3];
    elevator_cmd = control_norm[4];
    throttle_cmd = control_norm[2];
    rudder_cmd = control_norm[5];
    flap_cmd = control_norm[6];
    gear_cmd = control_norm[7];
        
    // mixing modes that work at the 'command' level (before actuator
    // value assignment)
    if ( config_actuators.mix_autocoord ) {
        rudder_cmd += config_actuators.mix_Gac * aileron_cmd;
    }
    if ( config_actuators.mix_throttle_trim ) {
        elevator_cmd += config_actuators.mix_Get * throttle_cmd;
    }
    if ( config_actuators.mix_flap_trim ) {
        elevator_cmd += config_actuators.mix_Gef * flap_cmd;
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
    if ( config_actuators.mix_elevon ) {
        actuator_norm[1] = config_actuators.mix_Gea * aileron_cmd + config_actuators.mix_Gee * elevator_cmd;
        actuator_norm[2] = config_actuators.mix_Gea * aileron_cmd - config_actuators.mix_Gee * elevator_cmd;
    } else if ( config_actuators.mix_flaperon ) {
        actuator_norm[1] = config_actuators.mix_Gfa * aileron_cmd + config_actuators.mix_Gff * flap_cmd;
        actuator_norm[4] = -config_actuators.mix_Gfa * aileron_cmd + config_actuators.mix_Gff * flap_cmd;
    }
    // vtail mixing can't work with elevon mixing
    if ( config_actuators.mix_vtail && !config_actuators.mix_elevon) {
        actuator_norm[2] = config_actuators.mix_Gve * elevator_cmd + config_actuators.mix_Gvr * rudder_cmd;
        actuator_norm[3] = config_actuators.mix_Gve * elevator_cmd - config_actuators.mix_Gvr * rudder_cmd;
    }
    if ( config_actuators.mix_diff_thrust ) {
        // fixme: never tested in the wild (need to think through channel assignments)
        // actuator_norm[0] = config_actuators.mix_Gtt * throttle_cmd + config_actuators.mix_Gtr * rudder_cmd;
        // actuator_norm[5] = config_actuators.mix_Gtt * throttle_cmd - config_actuators.mix_Gtr * rudder_cmd;
    }

    // compute pwm actuator output values from the normalized values
    pwm_norm2pwm( actuator_norm, actuator_pwm );
}

// set default raw actuator values
void actuator_set_defaults() {
    for ( int i = 0; i < SBUS_CHANNELS; i++ ) {
        actuator_norm[i] = 0.0;
    }
    pwm_norm2pwm(actuator_norm, actuator_pwm);
}
