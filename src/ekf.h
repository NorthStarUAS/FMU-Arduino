// ins wrapper class

#pragma once

#include <Arduino.h>

#include "../setup_board.h"
#include "nav_common/structs.h"

#if defined(AURA_ONBOARD_EKF)
#include "nav_ekf15/EKF_15state.h"
#include "nav_ekf15_mag/EKF_15state.h"
#endif

class ekf_t {
private:
    bool ekf_inited = false;
    unsigned long int gps_last_millis = 0;
#if defined(AURA_ONBOARD_EKF)
    EKF15 ekf;
    EKF15_mag ekf_mag;
#endif
    
public:
    NAVdata nav;
    uint8_t status;             // 0 = uninitted, 1 = no gps, 2 = 0k
    void setup();
    void update();
    void reinit();              // request the filter reinit itself
};

extern ekf_t ekf;
