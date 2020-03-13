// GPS wrapper class

#pragma once

#include "sensors/UBLOX8/UBLOX8.h"

class gps_t {
public:
    ublox8_nav_pvt_t gps_data;
    bool new_gps_data = false;
    void setup();
    void update();
};

extern gps_t gps;
