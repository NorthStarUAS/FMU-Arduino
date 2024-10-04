#pragma once

#include <math.h>
#include "eigen.h"

#include "../util/lowpass.h"

class calib_accels_t {

private:

    uint32_t last_millis = 0;
    int state = -1;
    bool armed = false;
    LowPassFilter ax_slow = LowPassFilter(1.0);
    LowPassFilter ax_fast = LowPassFilter(0.2);
    LowPassFilter ay_slow = LowPassFilter(1.0);
    LowPassFilter ay_fast = LowPassFilter(0.2);
    LowPassFilter az_slow = LowPassFilter(1.0);
    LowPassFilter az_fast = LowPassFilter(0.2);

    Eigen:: MatrixXf Ref;
    Eigen::MatrixXf Meas;
    bool checked[6] = {false};

    int raw_up_axis( float ax, float ay, float az );
    inline bool new_axis(int axis) {
        if ( axis >= 0 and not checked[axis] ) {
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
