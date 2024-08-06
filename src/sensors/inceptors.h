#pragma once

#include "../props2.h"
#include "../nodes.h"
#include "../comms/ns_messages.h"
#include "../../setup_board.h"
#include "sbus/sbus.h"
#include "switches.h"

class inceptors_t {

private:

    uint32_t last_input = 0;

public:

    switches_t switches;
    sbus_t sbus;

    void init();
    bool read();

};