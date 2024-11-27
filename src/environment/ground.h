#pragma once

#include "../util/lowpass.h"

class ground_est_t {

private:

    LowPassFilter airdata_ground_alt;
    LowPassFilter nav_ground_alt;
    bool ground_inited = false;

public:

    void init();
    void update(float dt);

};
