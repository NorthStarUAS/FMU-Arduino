#pragma once

#include "../props2.h"
#include "../nodes.h"
#include "../mixer.h"
#include "../comms/ns_messages.h"
#include "../sensors/sbus/sbus.h"
#include "../../setup_board.h"
#include "../sensors/pwm.h"
#include "../sensors/sbus/sbus.h"

class inceptors_t {

private:

    PropertyNode config_eff_gains_node;
    uint32_t last_input = 0;

public:

    mixer_t mixer;

    void init();
    bool read();
    void write();

};

extern inceptors_t inceptors;
