#pragma once

#include "aura4_messages.h"
#include "sbus.h"

class pilot_t {
private:
    
public:
    float manual_inputs[SBUS_CHANNELS]; // normalized
    float ap_inputs[SBUS_CHANNELS];    // normalized
    void setup();
    void update_manual();
    void update_ap( message::command_inceptors_t *inceptors );
    inline bool ap_enabled() { return manual_inputs[0] >= 0.0; }
    inline bool throttle_safety() { return manual_inputs[1] < 0.0; }
   
};

extern pilot_t pilot;
