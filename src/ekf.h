// ins wrapper class

#pragma once

#include <Arduino.h>

#include "EKF15/EKF_15state.h"

class ekf_t {
private:
    bool gps_found = false;
    bool ekf_inited = false;
    elapsedMillis gpsSettle = 0;
    EKF15 ekf;
    
public:
    NAVdata nav;
    void setup();
    void update();
};

extern ekf_t ekf;
