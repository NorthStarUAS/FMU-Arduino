#include "../../nodes.h"
#include "../../fcs/fcs_mgr.h"
#include "idle.h"

idle_task_t::idle_task_t(PropertyNode config_node) {
    name = "idle";
}

void idle_task_t::activate() {
    active = true;

    // set modes
    if ( !airdata_node.getBool("is_airborne") ) {
        fcs_mgr->set_mode("basic");
        mission_node.setString("mode", "none");
        controls_node.setDouble("power", 0.0);
    }
}

void idle_task_t::update(float dt) {
    if ( airdata_node.getBool("is_airborne") ) {
        // fixme: schedule a circle hold if we are airborne
    }
}

bool idle_task_t::is_complete() {
    return false;
}

void idle_task_t::close() {
    active = false;
}