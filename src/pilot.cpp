#include "pilot.h"

void pilot_t::setup() {
    manual_inputs[0] = ap_inputs[0] = -1.0; // autopilot disabled (manual)
    manual_inputs[1] = ap_inputs[1] = -1.0; // throttle safety enabled
    for ( int i = 2; i < SBUS_CHANNELS; i++ ) {
        manual_inputs[i] = ap_inputs[i] = 0.0;
    }
}

void pilot_t::update_manual() {
    sbus.raw2norm(manual_inputs);
}

void pilot_t::update_ap( message::command_inceptors_t *inceptors ) {
    // ap_inputs uses the same channel mapping as manual_inputs, so map
    // ap_tmp values to their correct places in autopilot_norm
    ap_inputs[0] = manual_inputs[0];      // auto/manual switch
    ap_inputs[1] = manual_inputs[1];      // throttle enable
    ap_inputs[2] = inceptors->channel[0]; // throttle
    ap_inputs[3] = inceptors->channel[1];
    ap_inputs[4] = inceptors->channel[2];
    ap_inputs[5] = inceptors->channel[3];
    ap_inputs[6] = inceptors->channel[4];
    ap_inputs[7] = inceptors->channel[5];
}

// global shared instance
pilot_t pilot;
