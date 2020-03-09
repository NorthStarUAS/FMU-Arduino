#pragma once

#include "aura4_messages.h"
#include "sensors/sbus/sbus.h"

class pilot_t {
public:
    float manual_inputs[SBUS_CHANNELS]; // normalized
    float ap_inputs[SBUS_CHANNELS];    // normalized
    
    void setup();
    void update_manual();
    void update_ap( message::command_inceptors_t *inceptors );

    // convenience
    inline bool ap_enabled() { return manual_inputs[0] >= 0.0; }
    inline bool throttle_safety() { return manual_inputs[1] < 0.0; }
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
};

extern pilot_t pilot;
