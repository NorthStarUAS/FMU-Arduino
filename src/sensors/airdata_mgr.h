// Airdata class

#pragma once

// #include "aura4_messages.h"

#include "../props2.h"

class airdata_mgr_t {

 private:

    PropertyNode airdata_node;
    PropertyNode config_node;

    uint8_t barometer = 0;
    uint8_t pitot = 0;
    bool pitot_found = false;
    bool ams_baro_found = false;

 public:

    float baro_press = 0.0;
    float baro_temp = 0.0;
    float baro_hum = 0.0;
    int error_count = 0;
    float diffPress_pa = 0.0;
    float temp_C = 0.0;

    void init();
    void update();
};

extern airdata_mgr_t airdata_mgr;
