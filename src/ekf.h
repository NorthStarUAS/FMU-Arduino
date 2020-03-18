// ins wrapper class

#pragma once

#include <Arduino.h>

#include "EKF15/EKF_15state.h"

class ekf_t {
private:
    bool ekf_inited = false;
    unsigned long int gps_last_millis = 0;
    EKF15 ekf;
    
public:
    NAVdata nav;
    void setup();
    void update();
    void reinit();              // request the filter reinit itself
};

extern ekf_t ekf;
