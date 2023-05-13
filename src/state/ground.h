#pragma once

#include "../util/lowpass.h"

class ground_est_t {

private:

    rcLowPassFilter ground_alt_filt;
    bool ground_alt_calibrated = false;

public:

    void init();
    void update(float dt);

};
