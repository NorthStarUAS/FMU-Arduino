#include "../../nodes.h"
#include "../../comms/events.h"
#include "../../fcs/fcs_mgr.h"
#include "../mission_mgr.h"
#include "circle.h"

circle_task_t::circle_task_t(PropertyNode config_node) {
    // name = config_node.getString("name");
    // if ( config_node.hasChild("direction") ) {
    //     // only override the default if a value is given
    //     direction = config_node.getString("direction");
    // }
    // if ( config_node.hasChild("radius_m") ) {
    //     // only override the default if a value is given
    //     radius_m = config_node.getDouble("radius_m");
    // }
    name = "circle";
}

void circle_task_t::activate() {
    active = true;

    // save current state
    // mission_mgr->state.save(true, true, false);

    // update_parameters();

    // set modes
    fcs_mgr->set_mode("basic+tecs");
    mission_node.setString("mode", "circle");
    event_mgr->add_event("mission", "circle");
}

void circle_task_t::update(float dt) {
    // circle hold is one of the two fundamental modes so there is nothing to
    // update here once the task is setup and activated.
}

bool circle_task_t::is_complete() {
    return false;
}

void circle_task_t::close() {
    // restore previous state
    // mission_mgr->state.restore();

    active = false;
}