// GPS wrapper class

#pragma once

#include "UBLOX8/UBLOX8.h"

#include <math.h>
#if defined(ARDUINO)
# include <eigen.h>
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <Eigen/LU>
using namespace Eigen;

#include "../props2.h"

class gps_mgr_t {

public:

    unsigned long gps_millis = 0;
    bool gps_acquired = false;
    bool gps_settled = false;
    elapsedMillis gps_settle_timer = 0;
    uint64_t unix_usec;
    float magvar_rad;
    Vector3f mag_ned;

    void setup();
    void update();

private:

    PropertyNode gps_node;
    ublox8_nav_pvt_t gps_data;

    void update_unix_usec();
    void update_magvar();
};

extern gps_mgr_t gps_mgr;