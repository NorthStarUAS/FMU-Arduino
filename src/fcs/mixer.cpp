// Module to handle actuator input/output and mixing.

#include "../../setup_board.h"
#include "../props2.h"
#include "../nodes.h"
#include "../sensors/pwm.h"

#include "mixer.h"

// reset sas parameters to startup defaults
void mixer_t::sas_defaults() {
    stab_roll_node.setBool("enable", true);
    stab_pitch_node.setBool("enable", true);
    stab_yaw_node.setBool("enable", true);
    stab_tune_node.setBool("false", true);
    stab_roll_node.setDouble("gain", 0.2);
    stab_pitch_node.setDouble("gain", 0.2);
    stab_yaw_node.setDouble("gain", 0.2);
};


void mixer_t::update_matrix() {
    M.setIdentity();            // straight pass through default

    // note: M(output_channel, input_channel)
    // note: elevon and flaperon mixing are mutually exclusive

    PropertyNode autocoord_node = PropertyNode("/config/mixer/auto_coordination");
    PropertyNode throttletrim_node = PropertyNode("/config/mixer/throttle_trim");
    PropertyNode flaptrim_node = PropertyNode("/config/mixer/flap_trim");
    PropertyNode elevon_node = PropertyNode("/config/mixer/elevon");
    PropertyNode flaperon_node = PropertyNode("/config/mixer/flaperon");
    PropertyNode vtail_node = PropertyNode("/config/mixer/vtail");
    PropertyNode diffthrust_node = PropertyNode("/config/mixer/diff_thrust");
    delay(100);

    if ( autocoord_node.getBool("enable") ) {
        M(3,1) = autocoord_node.getDouble("gain1");
        if (vtail_node.getBool("enable") && !elevon_node.getBool("enable")) {
            M(2,1) = -autocoord_node.getDouble("gain1");
        }
    }
    if ( throttletrim_node.getBool("enable") ) {
        M(2,0) = throttletrim_node.getDouble("gain1");
    }
    if ( flaptrim_node.getBool("enable") ) {
        M(2,4) = flaptrim_node.getDouble("gain1");
        if ( vtail_node.getBool("enable") && !elevon_node.getBool("enable")) {
            M(3,4) = flaptrim_node.getDouble("gain1");
        }
    }
    if ( elevon_node.getBool("enable") ) {
        M(1,1) = elevon_node.getDouble("gain1");
        M(1,2) = elevon_node.getDouble("gain2");
        M(2,1) = elevon_node.getDouble("gain1");
        M(2,2) = -elevon_node.getDouble("gain2");
    } else if ( flaperon_node.getBool("enable") ) {
        M(1,1) = flaperon_node.getDouble("gain1");
        M(1,4) = flaperon_node.getDouble("gain2");
        M(4,1) = -flaperon_node.getDouble("gain1");
        M(4,4) = flaperon_node.getDouble("gain2");
    }
    // vtail mixing can't work with elevon mixing
    if ( vtail_node.getBool("enable") && !elevon_node.getBool("enable") ) {
        M(2,2) = vtail_node.getDouble("gain1");
        M(2,3) = vtail_node.getDouble("gain2");
        M(3,2) = vtail_node.getDouble("gain1");
        M(3,3) = -vtail_node.getDouble("gain2");
    }
    if ( diffthrust_node.getBool("eanbled") ) {
        // fixme: never tested in the wild (need to think through channel assignments)
        // outputs[0] = mix_config.mix_Gtt * throttle_cmd + mix_config.mix_Gtr * rudder_cmd;
        // outputs[5] = mix_config.mix_Gtt * throttle_cmd - mix_config.mix_Gtr * rudder_cmd;
    }
}

void mixer_t::print_mixer_matrix() {
    Serial.print("Mixer Matrix:\n");
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        Serial.print("  ");
        for ( int j = 0; j < PWM_CHANNELS; j++ ) {
            if ( M(i,j) >= 0 ) {
                Serial.print(" ");
            }
            Serial.print(M(i,j), 2);
        }
        Serial.println("");
    }
}

void mixer_t::init() {
    stab_roll_node = PropertyNode("/config/stability_damper/roll");
    stab_pitch_node = PropertyNode("/config/stability_damper/pitch");
    stab_yaw_node = PropertyNode("/config/stability_damper/yaw");
    stab_tune_node = PropertyNode("/config/stability_damper/pilot_tune");

    M.resize(PWM_CHANNELS, PWM_CHANNELS);
    M.setIdentity();
    update_matrix();
    print_mixer_matrix();

    inputs.resize(PWM_CHANNELS);
    outputs.resize(PWM_CHANNELS);

    inputs.setZero();
    outputs.setZero();

    delay(100);
}

// compute the stability damping in normalized command/input space
// which simplifies mixing later
void mixer_t::sas_update() {
    float tune = 1.0;
    float max_tune = 2.0;
    if ( stab_tune_node.getBool("enable") ) {
        tune = max_tune * inceptors_node.getDouble("aux2");
        if ( tune < 0.0 ) {
            tune = 0.0;
        } else if ( tune > max_tune ) {
            tune = max_tune;
        }
    }

    if ( stab_roll_node.getBool("enable") ) {
        inputs[1] -= tune * stab_roll_node.getDouble("gain")
            * imu_node.getDouble("p_rps");
    }
    if ( stab_pitch_node.getBool("enable") ) {
        inputs[2] += tune * stab_pitch_node.getDouble("gain")
            * imu_node.getDouble("q_rps");
    }
    if ( stab_yaw_node.getBool("enable") ) {
        inputs[3] -= tune * stab_yaw_node.getDouble("gain")
            * imu_node.getDouble("r_rps");
    }
}

// compute the actuator (servo) values for each channel.  Handle all
// the requested mixing modes here.
void mixer_t::mixing_update() {
    outputs = M * inputs;

    if ( not inceptors_node.getBool("throttle_safety") ) {
        outputs[0] = 0.0;
    }

    // publish
    effectors_node.setUInt("millis", millis());
    for ( int i = 0; i < PWM_CHANNELS; i++ ) {
        effectors_node.setDouble("channel", outputs[i], i);
    }
}

void mixer_t::update( PropertyNode input_node ) {
    // the pilot.get_* interface is smart to return manual
    // vs. autopilot depending on switch state.
    inputs << input_node.getDouble("power"), input_node.getDouble("roll"),
        input_node.getDouble("pitch"), input_node.getDouble("yaw"),
        input_node.getDouble("flaps"), input_node.getDouble("gear"),
        input_node.getDouble("aux1"), input_node.getDouble("aux2");

    sas_update();
    mixing_update();
}
