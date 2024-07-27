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

    // extend gain array with default value (1.0) if not provided in config file
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

        // printf("%d ", nchannels);
        // for ( uint8_t i = 0; i < 8; i++ ) {
        //     printf("%.2f ", sbus.pwm_val[i]);
        // }
        // printf("\n");

        if ( sbus.receiver_flags & 1 << 3 ) {
            // fixme: consider forcing at least some simple reversionary mode
            // such as power off, wings level (or a few degrees for gentle
            // turning to avoid flyaway too far) and pitch to zero degrees or
            // something safe/slow or tecs to minimum speed with throttle off?
            inceptors_node.setBool("failsafe", true); // bad situation when AP not enabled!
        } else {
            for ( uint8_t i = 0; i < SBUS_CHANNELS; i++ ) {
                rcin_node.setUInt("channel", sbus.pwm_val[i], i);
            }
            last_input = millis();
            inceptors_node.setUInt("millis", last_input);

            // logical values
            inceptors_node.setBool("failsafe", false); // good
            // pilot_node.setBool("ap_enabled", ap_enabled());
            // pilot_node.setBool("throttle_safety", throttle_safety());

            inceptors_node.setDouble("power", sbus.norm_val[2]);
            inceptors_node.setDouble("roll", sbus.norm_val[3]);
            inceptors_node.setDouble("pitch", sbus.norm_val[4]);
            inceptors_node.setDouble("yaw", sbus.norm_val[5]);
            inceptors_node.setDouble("flaps", sbus.norm_val[6]);
            inceptors_node.setDouble("gear", sbus.norm_val[7]);
            inceptors_node.setDouble("aux1", sbus.norm_val[8]);
            inceptors_node.setDouble("aux2", sbus.norm_val[9]);
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

// global shared instance
pilot_t pilot;
