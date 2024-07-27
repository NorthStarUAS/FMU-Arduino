#pragma once

#include "airdata_mgr.h"
#include "gps_mgr.h"
#include "imu_mgr.h"
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
    power_t power;

private:

};

extern sensor_mgr_t *sensor_mgr;