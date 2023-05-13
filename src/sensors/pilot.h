#pragma once

#include "../nodes.h"
#include "../mixer.h"
#include "../comms/rc_messages.h"
#include "../sensors/sbus/sbus.h"
#include "../../setup_board.h"
#include "pwm.h"
#include "sbus/sbus.h"

class pilot_t {

private:
    // define if a channel is symmetrical or not (i.e. mapped to [0,1] for
    // everything but throttle, flaps, gear
    // static const uint16_t rcin_symmetrical = ~(1 << 2 | 1 << 6 | 1 << 7);
    // static const uint16_t rcout_symmetrical = ~(1 << 0 | 1 << 4 | 1 << 5);

    // float rcin2norm(uint16_t pwm_val, uint8_t channel);
    // uint16_t norm2rcout(float norm_val, uint8_t channel);

    uint32_t last_input = 0;

    // convenience
    inline bool ap_enabled() {
        return switches_node.getBool("master_switch");
    }
    inline bool throttle_safety() {
        return switches_node.getBool("throttle_safety");
    }
    inline float get_aileron() {
        if ( ap_enabled() ) {
            return ap_inputs[3];
        } else {
            return manual_inputs[3];
        }
    }
    inline float get_elevator() {
        if ( ap_enabled() ) {
            return ap_inputs[4];
        } else {
            return manual_inputs[4];
        }
    }
    inline float get_throttle() {
        if ( ap_enabled() ) {
            return ap_inputs[2];
        } else {
            return manual_inputs[2];
        }
    }
    inline float get_rudder() {
        if ( ap_enabled() ) {
            return ap_inputs[5];
        } else {
            return manual_inputs[5];
        }
    }
    inline float get_flap() {
        if ( ap_enabled() ) {
            return ap_inputs[6];
        } else {
            return manual_inputs[6];
        }
    }
    inline float get_gear() {
        if ( ap_enabled() ) {
            return ap_inputs[7];
        } else {
            return manual_inputs[7];
        }
    }
    inline float get_aux1() {
        return manual_inputs[8];
    }
    inline float get_aux2() {
        return manual_inputs[9];
    }

public:
    uint16_t pwm_inputs[SBUS_CHANNELS];
    float manual_inputs[SBUS_CHANNELS]; // normalized
    float ap_inputs[SBUS_CHANNELS];     // normalized
    uint16_t pwm_outputs[PWM_CHANNELS];

    mixer_t mixer;

    void init();
    bool read();
    void write();

    void update_ap( rc_message::inceptors_v1_t *inceptors );

};

extern pilot_t pilot;
