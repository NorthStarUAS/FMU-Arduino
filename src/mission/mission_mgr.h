#include "circle_mgr.h"
#include "route_mgr.h"
#include "tasks/task.h"

class mission_mgr_t {

public:

    circle_mgr_t circle_mgr;
    route_mgr_t route_mgr;
    task_t *current_task = nullptr;

    void init();
    void update();

    void new_task(task_t *task);
    void request_task_circle(double lon_deg, double lat_deg);
};

extern mission_mgr_t *mission_mgr;