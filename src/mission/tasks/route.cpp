#include "../../nodes.h"
#include "../../comms/events.h"
#include "../../fcs/fcs_mgr.h"
#include "../mission_mgr.h"
#include "route.h"

route_task_t::route_task_t(PropertyNode config_node) {
    name = "route";
}

void route_task_t::activate() {
    active = true;

    // defaults
    route_node.setString("follow_mode", "leader");
    route_node.setString("start_mode", "first_wpt");
    route_node.setString("completion_mode", "loop");

    // set modes
    fcs_mgr->set_mode("basic+tecs");
    mission_node.setString("mode", "route");
    event_mgr->add_event("mission", "route");
}

void route_task_t::update(float dt) {
    // route following is one of the two fundamental modes so there is nothing
    // to update here once the task is setup and activated.
}

bool route_task_t::is_complete() {
    return false;
}

void route_task_t::close() {
    active = false;
}