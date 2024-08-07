#include "circle_mgr.h"
#include "route_mgr.h"
#include "tasks/task.h"

class mission_mgr_t {

public:

    circle_mgr_t circle_mgr;
    route_mgr_t route_mgr;
    task_t *current_task = nullptr;

    void init();
    void update(float dt);

    void process_command_request();
    void new_task(task_t *task);
    void start_circle_task(double lon_deg, double lat_deg);
    void start_idle_task();
};

extern mission_mgr_t *mission_mgr;