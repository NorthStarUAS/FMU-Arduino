#include "../nodes.h"

#include "sbus/sbus.h"
#include "inceptors.h"

// 982 - 2006 (frsky) / 1496
// static const uint16_t PWM_MIN = 982;
// static const uint16_t PWM_MAX = 2006;
// static const uint16_t PWM_CENTER = (PWM_MIN + PWM_MAX) / 2;
// static const uint16_t PWM_HALF_RANGE = PWM_MAX - PWM_CENTER;
// static const uint16_t PWM_RANGE = PWM_MAX - PWM_MIN;

void inceptors_t::init() {
    // setup the hardware inputs and outputs
    sbus.init();
    switches.init();
}

bool inceptors_t::read() {
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
        // Serial.println(sbus.receiver_flags, BIN);

        if ( sbus.receiver_flags & 1 << 3 ) {
            // super bad situation if this happens when AP not enabled!

            // fixme: consider forcing at least some simple reversionary mode
            // such as power off, wings level (or a few degrees for gentle
            // turning to avoid flyaway too far) and pitch to zero degrees or
            // something safe/slow or tecs to minimum speed with throttle off?
            inceptors_node.setBool("failsafe", true);
        } else {
            // rcin_node is for switches and setup/debugging RC transmitter
            for ( uint8_t i = 0; i < SBUS_CHANNELS; i++ ) {
                rcin_node.setUInt("channel", sbus.pwm_val[i], i);
            }

            last_input = millis();
            inceptors_node.setUInt("millis", last_input);
            inceptors_node.setBool("failsafe", false); // i.e. good
            inceptors_node.setDouble("power", sbus.norm_val[2]);
            inceptors_node.setDouble("roll", sbus.norm_val[3]);
            inceptors_node.setDouble("pitch", sbus.norm_val[4]);
            inceptors_node.setDouble("yaw", sbus.norm_val[5]);
            inceptors_node.setDouble("flaps", sbus.norm_val[6]);
            inceptors_node.setDouble("gear", sbus.norm_val[7]);
            inceptors_node.setDouble("aux1", sbus.norm_val[8]);
            inceptors_node.setDouble("aux2", sbus.norm_val[9]);

            switches.update();
        }
    }
    return new_input;
}