#pragma once

#include "../props2.h"
#include "../util/lowpass.h"

class ground_est_t {

private:

    PropertyNode airdata_node;
    PropertyNode gps_node;
    PropertyNode pos_node;
    rcLowPassFilter ground_alt_filt;
    bool ground_alt_calibrated = false;

public:

    void init();
    void update(float dt);

};
