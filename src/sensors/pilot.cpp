#include "../nodes.h"

#include "pwm.h"
#include "sbus/sbus.h"
#include "pilot.h"

// 982 - 2006 (frsky) / 1496
// static const uint16_t PWM_MIN = 982;
// static const uint16_t PWM_MAX = 2006;
// static const uint16_t PWM_CENTER = (PWM_MIN + PWM_MAX) / 2;
// static const uint16_t PWM_HALF_RANGE = PWM_MAX - PWM_CENTER;
// static const uint16_t PWM_RANGE = PWM_MAX - PWM_MIN;

void pilot_t::init() {
    config_eff_gains_node = PropertyNode("/config/pwm");

    // extend gain array with default value (1.0) if not provided in
    // config file
    uint8_t size = config_eff_gains_node.getLen("gains");
    for ( uint8_t i = size; i < PWM_CHANNELS; i++ ) {
        config_eff_gains_node.setDouble("gains", 1.0, i);
    }

    // setup the hardware inputs and outputs
    pwm.init(-1); // fixme need to specify board from config file (which may not have it since we are porting from ardupilot devel environment)
    sbus.init();
    mixer.init();
}

bool pilot_t::read() {
    bool new_input = false;
    while ( sbus.process() ) {
        new_input = true;
    }

    if ( new_input ) {
        // sbus is expected to continue outputing data after transmitter signal
        // is lost.
        if ( sbus.receiver_flags & 1 << 3 ) {
            // fixme: consider forcing at least some simple reversionary mode
            // such as power off, wings level (or a few degrees for gentle
            // turning to avoid flyaway too far) and pitch to zero degrees or
            // something safe/slow or tecs to minimum speed with throttle off?
            pilot_node.setBool("failsafe", true); // bad situation
        } else {
            for ( uint8_t i = 0; i < SBUS_CHANNELS; i++ ) {
                rcin_node.setUInt("channel", sbus.pwm_val[i], i);
                pilot_node.setDouble("channel", sbus.norm_val[i], i);
                manual_inputs[i] = sbus.norm_val[i];
            }
            last_input = millis();
            pilot_node.setUInt("millis", last_input);
            // logical values
            pilot_node.setBool("failsafe", false); // good
            // pilot_node.setBool("ap_enabled", ap_enabled());
            // pilot_node.setBool("throttle_safety", throttle_safety());
            pilot_node.setDouble("aileron", get_aileron());
            pilot_node.setDouble("elevator", get_elevator());
            pilot_node.setDouble("throttle", get_throttle());
            pilot_node.setDouble("rudder", get_rudder());
            pilot_node.setDouble("flaps", get_flap());
            pilot_node.setDouble("gear", get_gear());
            pilot_node.setDouble("aux1", get_aux1());
            pilot_node.setDouble("aux2", get_aux2());
            // printf("%d ", nchannels);
            // for ( uint8_t i = 0; i < 8; i++ ) {
            //     printf("%.2f ", sbus.pwm_val[i]);
            // }
            // printf("\n");
        }
    }
    return new_input;
}

void pilot_t::write() {
    // available inputs have been parsed/sorted so do the mixing right
    // before outputing the effector commands.
    mixer.update();

    /*printf("safety: %d armed: %d\n", hal.util->safety_switch_state(), hal.util->get_soft_armed());*/

    for ( uint8_t i = 0; i < PWM_CHANNELS; i++ ) {
        // float norm_val = mixer.outputs[i] * config.pwm_cfg.act_gain[i];
        float norm_val = effectors_node.getDouble("channel", i)
            * config_eff_gains_node.getDouble("gains", i);
        uint16_t pwm_val = pwm.norm2pwm(norm_val, i);
        pwm.output_pwm[i] = pwm_val;
        // printf("%d ", pwm_val);
    }
    pwm.write();
    // printf("\n");
    // pwm_test = 1000 + (AP_HAL::millis() % 5000) / 5;
    // for ( uint8_t i = MAX_RCOUT_CHANNELS; i < 14; i++ ) {
    //     hal.rcout->write(i, pwm_test);
    // }
}

void pilot_t::update_ap( ns_message::inceptors_v1_t *inceptors ) {
    // ap_inputs uses the same channel mapping as manual_inputs, so map
    // ap_tmp values to their correct places
    ap_inputs[0] = manual_inputs[0];      // auto/manual switch
    ap_inputs[1] = manual_inputs[1];      // throttle enable
    ap_inputs[2] = inceptors->channel[0]; // throttle
    ap_inputs[3] = inceptors->channel[1]; // aileron
    ap_inputs[4] = inceptors->channel[2]; // elevator
    ap_inputs[5] = inceptors->channel[3]; // rudder
    ap_inputs[6] = inceptors->channel[4]; // flap
    ap_inputs[7] = inceptors->channel[5]; // gear
    for ( int i = 0; i < SBUS_CHANNELS; i++ ) {
        pilot_node.setDouble("auto", ap_inputs[i], i);
    }
}

// global shared instance
pilot_t pilot;
