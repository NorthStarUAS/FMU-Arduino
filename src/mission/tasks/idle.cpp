#include <string>
using std::string;

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
    // if we find ourselves airborne and idle (and know our position) switch to
    // a circle task
    if ( airdata_node.getBool("is_airborne") ) {
        if ( gps_node.getInt("status") == 3 ) {
            mission_node.setString("request", "circle_here");
        }
    }
}

bool idle_task_t::is_complete() {
    return false;
}

void idle_task_t::close() {
    active = false;
}