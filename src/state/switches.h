#pragma once

#include "../../setup_board.h"
#include "../props2.h"

struct switch_t {
    uint8_t rc_channel = 0;
    string name = "";
    uint8_t num_states = 2;
    int value = 0;
    int last_state = -1;
};

class switches_t {

private:

    PropertyNode config_node;
    PropertyNode rcin_node;
    PropertyNode switches_node;

    vector<switch_t> switch_list;

public:

    void init();
    void update();
};
