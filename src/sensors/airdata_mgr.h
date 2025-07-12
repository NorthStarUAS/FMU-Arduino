// Airdata class

#pragma once

#include "../nodes.h"
#include "../../setup_board.h"
#include "../util/butter.h"

class airdata_mgr_t {

private:

    PropertyNode config_airdata_node;

    uint8_t barometer = 0;
    uint8_t pitot = 0;
    bool pitot_found = false;
    bool ams_baro_found = false;
    bool airspeed_inited = false;
    uint32_t airspeed_init_start_millis = 0;
    float pitot_sum = 0.0;
    uint32_t pitot_count = 0;
    float pitot_offset = 0.0;

    // 2nd order filter, MASTER_HZ (main update loop rate) sample rate, 3rd
    // field is cutoff freq.  higher freq value == noisier, a value near 1 hz
    // should work well for airspeed, will start with a similar value for
    // altitude.
    ButterworthFilter diff_press_butter = ButterworthFilter(2, MASTER_HZ, 0.8);
    ButterworthFilter static_press_butter = ButterworthFilter(2, MASTER_HZ, 1.0);

    void compute_altitude();
    void compute_airspeed();

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