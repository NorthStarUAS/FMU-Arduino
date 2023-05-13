#pragma once

#include "../util/lowpass.h"

class wind_est_t {

private:

    rcLowPassFilter we_filt;
    rcLowPassFilter wn_filt;
    rcLowPassFilter pitot_scale_filt;

public:

    void init();
    void update( double dt );

};
