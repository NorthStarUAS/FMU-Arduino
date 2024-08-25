#pragma once

#include "../../props2.h"
#include "task.h"

class launch_task_t : public task_t {

public:

    launch_task_t( PropertyNode config_node );
    ~launch_task_t() {}

    void activate();
    void update(float dt);
    bool is_complete();
    void close();

private:

    string launch_mode = "hand";  // "hand" or "surface"
    float completion_agl_ft = 150.0;
    float mission_agl_ft = 300.0;
    float ref_airspeed_kt = 15;  // FIXME: update config file too!
    float ref_pitch_deg = 10;
    float roll_gain = 0.5;
    float roll_limit = 5.0;
    bool rudder_enable = false;
    float rudder_gain = 1.0;
    float rudder_max = 1.0;
    float control_limit = 1.0;
    float power = 0.0;
    float flaps = 0.0;

    bool last_ap_master = false;
    float relhdg = 0.0;

};