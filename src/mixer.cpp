// Module to handle actuator input/output and mixing.

#include "config.h"
#include "imu.h"
#include "pilot.h"

#include "mixer.h"

// reset actuator gains (reversing) to startup defaults
void mixer_t::act_gain_defaults() {
    for ( int i = 0; i < message::pwm_channels; i++ ) {
        config.actuators.act_gain[i] = 1.0;
    }
}

// reset sas parameters to startup defaults
void mixer_t::sas_defaults() {
    config.actuators.sas_rollaxis = false;
    config.actuators.sas_pitchaxis = false;
    config.actuators.sas_yawaxis = false;
    config.actuators.sas_tune = false;

    config.actuators.sas_rollgain = 0.0;
    config.actuators.sas_pitchgain = 0.0;
    config.actuators.sas_yawgain = 0.0;
    config.actuators.sas_max_gain = 2.0;
};


// reset mixing parameters to startup defaults
void mixer_t::mixing_defaults() {
    config.actuators.mix_autocoord = false;
    config.actuators.mix_throttle_trim = false;
    config.actuators.mix_flap_trim = false;
    config.actuators.mix_elevon = false;
    config.actuators.mix_flaperon = false;
    config.actuators.mix_vtail = false;
    config.actuators.mix_diff_thrust = false;

    config.actuators.mix_Gac = 0.5;       // aileron gain for autocoordination
    config.actuators.mix_Get = -0.1;      // elevator trim w/ throttle gain
    config.actuators.mix_Gef = 0.1;       // elevator trim w/ flap gain

    config.actuators.mix_Gea = 1.0;       // aileron gain for elevons
    config.actuators.mix_Gee = 1.0;       // elevator gain for elevons
    config.actuators.mix_Gfa = 1.0;       // aileron gain for flaperons
    config.actuators.mix_Gff = 1.0;       // flaps gain for flaperons
    config.actuators.mix_Gve = 1.0;       // elevator gain for vtail
    config.actuators.mix_Gvr = 1.0;       // rudder gain for vtail
    config.actuators.mix_Gtt = 1.0;       // throttle gain for diff thrust
    config.actuators.mix_Gtr = 0.1;       // rudder gain for diff thrust
};


void mixer_t::setup() {
    mixing_defaults();
    M.setIdentity();            // straight pass through default
    outputs.setZero();
    pwm.norm2pwm( outputs.data() );
    
    // mixing modes that work at the 'command' level (before actuator
    // value assignment)
    if ( config.actuators.mix_autocoord ) {
        M(3,1) = config.actuators.mix_Gac;
    }
    if ( config.actuators.mix_throttle_trim ) {
        M(2,0) = config.actuators.mix_Get;
    }
    if ( config.actuators.mix_flap_trim ) {
        M(2,4) = config.actuators.mix_Gef;
    }

    // elevon and flaperon mixing are mutually exclusive
    if ( config.actuators.mix_elevon ) {
        M(1,1) = config.actuators.mix_Gea;
        M(1,2) = config.actuators.mix_Gee;
        M(2,1) = config.actuators.mix_Gea;
        M(2,2) = -config.actuators.mix_Gee;
    } else if ( config.actuators.mix_flaperon ) {
        M(1,1) = config.actuators.mix_Gfa;
        M(1,4) = config.actuators.mix_Gff;
        M(4,1) = -config.actuators.mix_Gfa;
        M(4,4) = config.actuators.mix_Gff;
    }
    // vtail mixing can't work with elevon mixing
    if ( config.actuators.mix_vtail && !config.actuators.mix_elevon) {
        M(2,2) = config.actuators.mix_Gve;
        M(2,3) = config.actuators.mix_Gvr;
        M(3,2) = config.actuators.mix_Gve;
        M(3,3) = -config.actuators.mix_Gvr;
    }
    if ( config.actuators.mix_diff_thrust ) {
        // fixme: never tested in the wild (need to think through channel assignments)
        // outputs[0] = config.actuators.mix_Gtt * throttle_cmd + config.actuators.mix_Gtr * rudder_cmd;
        // outputs[5] = config.actuators.mix_Gtt * throttle_cmd - config.actuators.mix_Gtr * rudder_cmd;
    }
    
    Serial.println("Mixer Matrix:");
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        for ( int j = 0; j < PWM_CHANNELS; j++ ) {
            Serial.print(M(i,j), 2);
            Serial.print(" ");
        }
        Serial.println();
    }
}

// compute the stability damping in normalized command/input space
// which simplifies mixing later
void mixer_t::sas_update() {
    float tune = 1.0;
    if ( config.actuators.sas_tune ) {
        tune = config.actuators.sas_max_gain * pilot.manual_inputs[7];
        if ( tune < 0.0 ) {
            tune = 0.0;
        } else if ( tune > 2.0 ) {
            tune = 2.0;
        }
    }

    if ( config.actuators.sas_rollaxis ) {
        inputs[1] -= tune * config.actuators.sas_rollgain * imu.get_p();
    }
    if ( config.actuators.sas_pitchaxis ) {
        inputs[2] += tune * config.actuators.sas_pitchgain * imu.get_q();
    }
    if ( config.actuators.sas_yawaxis ) {
        inputs[3] += tune * config.actuators.sas_yawgain * imu.get_r();
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
