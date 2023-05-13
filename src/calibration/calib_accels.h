#pragma once

#include <math.h>
#include "eigen.h"

#include "../util/lowpass.h"

class calib_accels_t {

private:

    uint32_t last_millis = 0;
    int state = -1;
    bool armed = false;
    rcLowPassFilter ax_slow = rcLowPassFilter(1.0);
    rcLowPassFilter ax_fast = rcLowPassFilter(0.2);
    rcLowPassFilter ay_slow = rcLowPassFilter(1.0);
    rcLowPassFilter ay_fast = rcLowPassFilter(0.2);
    rcLowPassFilter az_slow = rcLowPassFilter(1.0);
    rcLowPassFilter az_fast = rcLowPassFilter(0.2);

    Eigen:: MatrixXf Ref;
    Eigen::MatrixXf Meas;
    bool checked[6] = {false};

    int raw_up_axis( float ax, float ay, float az );
    inline bool new_axis(int axis) {
        if ( axis >= 0 and !checked[axis] ) {
            return true;
        } else {
            return false;
        }
    }
    float fit_metrics(Eigen::MatrixXf affine);

public:

    void init();
    void update();

};
