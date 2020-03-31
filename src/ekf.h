// ins wrapper class

#pragma once

#include <Arduino.h>

#include "nav_ekf15/EKF_15state.h"

class ekf_t {
private:
    bool ekf_inited = false;
    unsigned long int gps_last_millis = 0;
    EKF15 ekf;
    
public:
    NAVdata nav;
    uint8_t status;             // 0 = uninitted, 1 = no gps, 2 = 0k
    void setup();
    void update();
    void reinit();              // request the filter reinit itself
};

extern ekf_t ekf;
