// Module to handle actuator input/output and mixing.

#include "imu.h"
#include "pilot.h"

#include "mixer.h"

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


void mixer_t::setup() {
    mixing_defaults();
    M.setIdentity();            // straight pass through default

    // mixing modes that work at the 'command' level (before actuator
    // value assignment)
    if ( config.mix_autocoord ) {
        M(3,1) = config.mix_Gac;
    }
    if ( config.mix_throttle_trim ) {
        M(2,0) = config.mix_Get;
    }
    if ( config.mix_flap_trim ) {
        M(2,4) = config.mix_Gef;
    }

    // elevon and flaperon mixing are mutually exclusive
    if ( config.mix_elevon ) {
        M(1,1) = config.mix_Gea;
        M(1,2) = config.mix_Gee;
        M(2,1) = config.mix_Gea;
        M(2,2) = -config.mix_Gee;
    } else if ( config.mix_flaperon ) {
        M(1,1) = config.mix_Gfa;
        M(1,4) = config.mix_Gff;
        M(4,1) = -config.mix_Gfa;
        M(4,4) = config.mix_Gff;
    }
    // vtail mixing can't work with elevon mixing
    if ( config.mix_vtail && !config.mix_elevon) {
        M(2,2) = config.mix_Gve;
        M(2,3) = config.mix_Gvr;
        M(3,2) = config.mix_Gve;
        M(3,3) = -config.mix_Gvr;
    }
    if ( config.mix_diff_thrust ) {
        // fixme: never tested in the wild (need to think through channel assignments)
        // outputs[0] = config.mix_Gtt * throttle_cmd + config.mix_Gtr * rudder_cmd;
        // outputs[5] = config.mix_Gtt * throttle_cmd - config.mix_Gtr * rudder_cmd;
    }
    
    Serial.println("Mixer Matrix:");
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        for ( int j = 0; j < PWM_CHANNELS; j++ ) {
            Serial.print(M(i,j), 2);
            Serial.print(" ");
        }
        Serial.println();
    }
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        outputs[i] = 0.0;
    }
}

// compute the stability damping in normalized command/input space
// which simplifies mixing later
void mixer_t::sas_update() {
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
        inputs[1] -= tune * config.sas_rollgain * imu.get_p();
    }
    if ( config.sas_pitchaxis ) {
        inputs[2] += tune * config.sas_pitchgain * imu.get_q();
    }
    if ( config.sas_yawaxis ) {
        inputs[3] += tune * config.sas_yawgain * imu.get_r();
    }
}

// compute the actuator (servo) values for each channel.  Handle all
// the requested mixing modes here.
void mixer_t::mixing_update() {
    outputs = M * inputs;
    
    if ( pilot.throttle_safety() ) {
        outputs[0] = 0.0;
    }
}

void mixer_t::update( float control_norm[SBUS_CHANNELS] ) {
    // initialize commands
    inputs << pilot.get_throttle(), pilot.get_aileron(), pilot.get_elevator(),
        pilot.get_rudder(), pilot.get_flap(), pilot.get_gear(),
        pilot.get_ch7(), pilot.get_ch8();
    
    sas_update();
    mixing_update();

    // compute pwm actuator output values from the normalized values
    pwm.norm2pwm( outputs.data() );
}

// global shared instance
mixer_t mixer;
