#pragma once

#include "../props2.h"
#include "../nodes.h"
#include "../mixer.h"
#include "../comms/ns_messages.h"
#include "../sensors/sbus/sbus.h"
#include "../../setup_board.h"
#include "pwm.h"
#include "sbus/sbus.h"

class pilot_t {

private:

    PropertyNode config_eff_gains_node;
    uint32_t last_input = 0;

public:

    mixer_t mixer;

    void init();
    bool read();
    void write();

};

extern pilot_t pilot;
