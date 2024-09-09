#pragma once

#include "../util/profile.h"
#include "airdata_mgr.h"
#include "gps_mgr.h"
#include "imu_mgr.h"
#include "inceptors.h"
#include "power.h"

class sensor_mgr_t {

public:

    sensor_mgr_t() {};
    ~sensor_mgr_t() {};
    void init();
    void update();

    airdata_mgr_t airdata_mgr;
    gps_mgr_t gps_mgr;
    imu_mgr_t imu_mgr;
    inceptors_t inceptors;
    power_t power;

};

extern sensor_mgr_t *sensor_mgr;