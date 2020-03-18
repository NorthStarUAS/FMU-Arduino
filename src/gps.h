// GPS wrapper class

#pragma once

#include "sensors/UBLOX8/UBLOX8.h"

class gps_t {
public:
    long gps_millis = 0;
    bool gps_acquired = false;
    elapsedMillis gps_alive = 0;
    ublox8_nav_pvt_t gps_data;

    void setup();
    void update();
    bool settle();
};

extern gps_t gps;
