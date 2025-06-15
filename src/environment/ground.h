// FIXME: merge this module with airdata

#pragma once

#include "../util/lowpass.h"

class ground_est_t {

private:

    LowPassFilter airdata_ground_alt;
    // LowPassFilter gps_ground_alt;
    bool ground_inited = false;

public:

    void init();
    void update(float dt);

};
