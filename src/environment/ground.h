// FIXME: merge this module with airdata

#pragma once

#include "../util/lowpass.h"

class ground_est_t {

private:

    LowPassFilter est_airdata_ground_m;
    // LowPassFilter gps_ground_alt;
    bool est_ground_inited = false;

public:

    void init();
    void update(float dt);

};
