#include "../nodes.h"

#include "../sensors/pwm.h"
#include "effectors.h"

// 982 - 2006 (frsky) / 1496
// static const uint16_t PWM_MIN = 982;
// static const uint16_t PWM_MAX = 2006;
// static const uint16_t PWM_CENTER = (PWM_MIN + PWM_MAX) / 2;
// static const uint16_t PWM_HALF_RANGE = PWM_MAX - PWM_CENTER;
// static const uint16_t PWM_RANGE = PWM_MAX - PWM_MIN;

void effectors_t::init() {
    config_eff_gains_node = PropertyNode("/config/pwm");

    // extend gain array with default value (1.0) if not provided in config file
    uint8_t size = config_eff_gains_node.getLen("gains");
    for ( uint8_t i = size; i < PWM_CHANNELS; i++ ) {
        config_eff_gains_node.setDouble("gains", 1.0, i);
    }

    // setup the hardware inputs and outputs
    pwm.init(-1); // fixme need to specify board from config file (which may not have it since we are porting from ardupilot devel environment)
    mixer.init();
}

void effectors_t::write( PropertyNode input_node ) {
    // available inputs have been parsed/sorted so do the mixing right
    // before outputing the effector commands.
    mixer.update(input_node);

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
