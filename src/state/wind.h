#pragma once

#include "../util/lowpass.h"

class wind_est_t {

private:

    LowPassFilter we_filt;
    LowPassFilter wn_filt;
    LowPassFilter pitot_scale_filt;

public:

    void init();
    void update( double dt );

};
