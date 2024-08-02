#include "circle_mgr.h"
#include "route_mgr.h"
#include "state.h"

class mission_mgr_t {

public:

    circle_mgr_t circle_mgr;
    route_mgr_t route_mgr;
    MissionState state;

    void init();
    void update();
};

extern mission_mgr_t *mission_mgr;