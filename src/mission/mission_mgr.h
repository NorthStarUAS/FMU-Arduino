#pragma once

#include "global/circle_mgr.h"
#include "global/home_mgr.h"
#include "global/route_mgr.h"
#include "global/throttle_safety.h"
#include "tasks/task.h"

class mission_mgr_t {

public:

    circle_mgr_t circle_mgr;
    home_mgr_t home_mgr;
    route_mgr_t route_mgr;
    throttle_safety_task_t throttle_safety;
    task_t *current_task = nullptr;

    void init();
    void update(float dt);

    void process_command_request();
    void new_task(task_t *task);
    void start_circle_task(double lon_deg, double lat_deg);
    void start_launch_task();
    void start_land_task();
    void start_route_task();
    void start_idle_task();

private:

    bool last_link_state = true;

};

extern mission_mgr_t *mission_mgr;