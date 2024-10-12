#pragma once

#include "../props2.h"
#include "../nodes.h"
#include "../../setup_board.h"
#include "switches.h"

// for normalizing sbus channel values
static const int SBUS_MIN_VALUE = 172;
static const int SBUS_CENTER_VALUE = 992;
static const int SBUS_MAX_VALUE = 1811;
static const int SBUS_RANGE = 1640;
static const int SBUS_HALF_RANGE = 820;

class inceptors_t {

private:

    uint32_t last_input = 0;

public:

    switches_t switches;

    void init();
    bool read();

};