#pragma once

#include "../../props2.h"
#include "task.h"

class land_task4_t : public task_t {

public:

    land_task4_t();
    ~land_task4_t() {}

    void activate();
    void build_approach(PropertyNode config_node);
    void update(float dt);
    bool is_complete();
    void close();

private:

    float alt_base_agl_ft = 0.0;
    float tan_gs = 0.0;
    float circle_radius_m = 0.0;
    float flare_pitch_deg = 0.0;
    float flare_seconds = 0.0;
    float side = -1.0;
    float flare_start_time = 0.0;
    float approach_power = 0.0;
    float approach_pitch = 0.0;
    float flare_pitch_range = 0.0;
    float final_heading_deg = 0.0;
    float final_leg_m = 0.0;
    float dist_rem_m = 0.0;
    bool circle_capture = false;
    bool gs_capture = false;
    bool in_flare = false;

};