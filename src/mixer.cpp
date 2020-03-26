// Module to handle actuator input/output and mixing.

#include "config.h"
#include "imu.h"
#include "pilot.h"

#include "mixer.h"

// reset sas parameters to startup defaults
void mixer_t::sas_defaults() {
    config.stab.sas_rollaxis = false;
    config.stab.sas_pitchaxis = false;
    config.stab.sas_yawaxis = false;
    config.stab.sas_tune = false;

    config.stab.sas_rollgain = 0.0;
    config.stab.sas_pitchgain = 0.0;
    config.stab.sas_yawgain = 0.0;
    config.stab.sas_max_gain = 2.0;
};


// // reset mixing parameters to startup defaults
// void mixer_t::mixing_defaults() {
//     config.actuators.mix_autocoord = false;
//     config.actuators.mix_throttle_trim = false;
//     config.actuators.mix_flap_trim = false;
//     config.actuators.mix_elevon = false;
//     config.actuators.mix_flaperon = false;
//     config.actuators.mix_vtail = false;
//     config.actuators.mix_diff_thrust = false;

//     config.actuators.mix_Gac = 0.5;       // aileron gain for autocoordination
//     config.actuators.mix_Get = -0.1;      // elevator trim w/ throttle gain
//     config.actuators.mix_Gef = 0.1;       // elevator trim w/ flap gain

//     config.actuators.mix_Gea = 1.0;       // aileron gain for elevons
//     config.actuators.mix_Gee = 1.0;       // elevator gain for elevons
//     config.actuators.mix_Gfa = 1.0;       // aileron gain for flaperons
//     config.actuators.mix_Gff = 1.0;       // flaps gain for flaperons
//     config.actuators.mix_Gve = 1.0;       // elevator gain for vtail
//     config.actuators.mix_Gvr = 1.0;       // rudder gain for vtail
//     config.actuators.mix_Gtt = 1.0;       // throttle gain for diff thrust
//     config.actuators.mix_Gtr = 0.1;       // rudder gain for diff thrust
// };


void mixer_t::update_matrix(message::config_mixer_t *mix_config ) {
    M.setIdentity();            // straight pass through default

    // note: M(output_channel, input_channel)
    // note: elevon and flaperon mixing are mutually exclusive
    
    if ( mix_config->mix_autocoord ) {
        M(3,1) = -mix_config->mix_Gac;
        if ( mix_config->mix_vtail && !mix_config->mix_elevon) {
            M(2,1) = mix_config->mix_Gac;
        }
    }
    if ( mix_config->mix_throttle_trim ) {
        M(2,0) = mix_config->mix_Get;
    }
    if ( mix_config->mix_flap_trim ) {
        M(2,4) = mix_config->mix_Gef;
        if ( mix_config->mix_vtail && !mix_config->mix_elevon) {
            M(3,4) = -mix_config->mix_Gef;
        }
    }
    if ( mix_config->mix_elevon ) {
        M(1,1) = mix_config->mix_Gea;
        M(1,2) = mix_config->mix_Gee;
        M(2,1) = mix_config->mix_Gea;
        M(2,2) = -mix_config->mix_Gee;
    } else if ( mix_config->mix_flaperon ) {
        M(1,1) = mix_config->mix_Gfa;
        M(1,4) = mix_config->mix_Gff;
        M(4,1) = -mix_config->mix_Gfa;
        M(4,4) = mix_config->mix_Gff;
    }
    // vtail mixing can't work with elevon mixing
    if ( mix_config->mix_vtail && !mix_config->mix_elevon) {
        M(2,2) = mix_config->mix_Gve;
        M(2,3) = mix_config->mix_Gvr;
        M(3,2) = mix_config->mix_Gve;
        M(3,3) = -mix_config->mix_Gvr;
    }
    if ( mix_config->mix_diff_thrust ) {
        // fixme: never tested in the wild (need to think through channel assignments)
        // outputs[0] = mix_config->mix_Gtt * throttle_cmd + mix_config->mix_Gtr * rudder_cmd;
        // outputs[5] = mix_config->mix_Gtt * throttle_cmd - mix_config->mix_Gtr * rudder_cmd;
    }

    // updating the mixer_matrix config message so we can save it in eeeprom
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        for ( int j = 0; j < PWM_CHANNELS; j++ ) {
            //Serial.println(j*PWM_CHANNELS+i);
            config.mixer_matrix.matrix[i*PWM_CHANNELS+j] = M(i,j);
        }
    }

    print_mixer_matrix();
}

void mixer_t::print_mixer_matrix() {
    Serial.println("Mixer Matrix:");
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        for ( int j = 0; j < PWM_CHANNELS; j++ ) {
            if ( M(i,j) >= 0 ) {
                Serial.print(" ");
            }
            Serial.print(M(i,j), 2);
            Serial.print(" ");
        }
        Serial.println();
    }
}
void mixer_t::setup() {
    outputs.setZero();
    pwm.norm2pwm( outputs.data() );
    M = Matrix<float, PWM_CHANNELS, PWM_CHANNELS, RowMajor>(config.mixer_matrix.matrix);
    print_mixer_matrix();
}

// compute the stability damping in normalized command/input space
// which simplifies mixing later
void mixer_t::sas_update() {
    float tune = 1.0;
    if ( config.stab.sas_tune ) {
        tune = config.stab.sas_max_gain * pilot.manual_inputs[7];
        if ( tune < 0.0 ) {
            tune = 0.0;
        } else if ( tune > 2.0 ) {
            tune = 2.0;
        }
    }

    if ( config.stab.sas_rollaxis ) {
        inputs[1] -= tune * config.stab.sas_rollgain * imu.get_p_cal();
    }
    if ( config.stab.sas_pitchaxis ) {
        inputs[2] += tune * config.stab.sas_pitchgain * imu.get_q_cal();
    }
    if ( config.stab.sas_yawaxis ) {
        inputs[3] += tune * config.stab.sas_yawgain * imu.get_r_cal();
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
