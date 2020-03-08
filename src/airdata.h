// Airdata class

#pragma once

#include "aura4_messages.h"

class airdata_t {
 private:
    bool pitot_found = false;
    bool ams_baro_found = false;
    
 public:
    message::config_airdata_t config;
    float baro_press = 0.0;
    float baro_temp = 0.0;
    float baro_hum = 0.0;
    int error_count = 0;
    float diffPress_pa = 0.0;
    float temp_C = 0.0;

    void defaults_none();
    void defaults_goldy3();
    void defaults_aura3();
    void setup();
    void update();
};

extern airdata_t airdata;
