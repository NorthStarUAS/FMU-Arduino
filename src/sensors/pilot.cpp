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
    config_eff_gains = PropertyNode("/config/pwm");
    effector_node = PropertyNode("/effectors");
    pilot_node = PropertyNode("/pilot");
    rcin_node = PropertyNode("/sensors/rc-input");
    switches_node = PropertyNode("/switches");

    // extend gain array with default value (1.0) if not provided in
    // config file
    uint8_t size = config_eff_gains.getLen("gains");
    for ( uint8_t i = size; i < PWM_CHANNELS; i++ ) {
        config_eff_gains.setDouble("gains", 1.0, i);
    }

    // setup the hardware inputs and outputs
    pwm.setup(-1); // fixme need to specify board from config file (which may not have it since we are porting from ardupilot devel environment)
    sbus.setup();

    mixer.setup();
}

bool pilot_t::read() {
    bool new_input = false;
    while ( sbus.process() ) {
        new_input = true;
    }

    if ( new_input ) {
        for ( uint8_t i = 0; i < SBUS_CHANNELS; i++ ) {
            rcin_node.setUInt("channel", sbus.pwm_val[i], i);
            pilot_node.setDouble("channel", sbus.norm_val[i], i);
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
    } else if ( millis() - last_input > 500 and !pilot_node.getBool("failsafe") ) {
        pilot_node.setBool("failsafe", true); // bad
        // printf("failsafe!\n");
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
        float norm_val = effector_node.getDouble("channel", i)
            * config_eff_gains.getDouble("gains", i);
        uint16_t pwm_val = pwm.norm2pwm(norm_val, i);
        pwm.output_pwm[i] = pwm_val;
        // printf("%d ", pwm_val);
        // hal.rcout->write(i, pwm_val);
    }
    pwm.write();
    // printf("\n");
    // pwm_test = 1000 + (AP_HAL::millis() % 5000) / 5;
    // for ( uint8_t i = MAX_RCOUT_CHANNELS; i < 14; i++ ) {
    //     hal.rcout->write(i, pwm_test);
    // }
}

void pilot_t::update_ap( rc_message::inceptors_v1_t *inceptors ) {
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
