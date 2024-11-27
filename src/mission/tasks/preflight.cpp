#include "../../nodes.h"
#include "../../fcs/fcs_mgr.h"
#include "preflight.h"

preflight_task_t::preflight_task_t() {
    name = "preflight";
    PropertyNode config_node = PropertyNode("/config/mission/preflight");
    if ( config_node.hasChild("duration_sec") ) {
        duration_sec = config_node.getDouble("duration_sec");
    }
}

void preflight_task_t::activate() {
    active = true;

    if ( not environment_node.getBool("is_airborne") ) {
        // set fcs mode to roll+pitch
        fcs_mgr->set_mode("roll+pitch");
        refs_node.setDouble("roll_deg", 0.0);
        refs_node.setDouble("pitch_deg", 0.0);
        refs_node.setDouble("flaps_setpoint", 0.0);
        // reset timer
        timer = 0.0;
    } else {
        // we are airborne, don't change modes and configure timer to be already
        // expired
        timer = duration_sec + 1.0;
    }
}

void preflight_task_t::update(float dt) {
    timer += dt;
}

bool preflight_task_t::is_complete() {
    // print "timer=%.1f duration=%.1f" % (self.timer, self.duration_sec)
    // complete when timer expires or we sense we are airborne (sanity check!)
    if ( timer >= duration_sec or environment_node.getBool("is_airborne") ) {
        return true;
    } else {
        return false;
    }
}

void preflight_task_t::close() {
    active = false;
}