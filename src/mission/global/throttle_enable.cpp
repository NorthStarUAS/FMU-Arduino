
#include "../../nodes.h"
#include "../../comms/events.h"
#include "throttle_enable.h"

// fixme: I forget exactly how this danced with the pilot transmitter command.
// Let's get the task ported and then figure that out!  Note sensors/switch.cpp
// only updates the switch state when the physical switch moves, so this can
// "force fight" with the transmitter state

void throttle_enable_task_t::init() {
    PropertyNode config_node = PropertyNode("/config/mission/throttle_enable");

    inceptors_node.setBool("throttle_enable", false);
    if ( config_node.hasChild("safety_mode") ) {
        safety_mode = config_node.getString("safety_mode");
    }
}

void throttle_enable_task_t::update() {
    bool is_airborne = airdata_node.getBool("is_airborne");
    if ( not airborne_latch and is_airborne ) {
        airborne_latch = true;
    }

    if ( not master_enable ) {
        // throttle disabled, check if we should enable it.
        if ( not gps_node.getBool("settle") ) {
            // do not enable throttle control if gps hasn't reported a fix and
            // is not settled.
        } else if ( safety_mode == "on_ground" and not is_airborne ) {
            // safety "on_ground" means never enable throttle unless we are
            // airborne.
        } else if ( safety_mode == "on_touchdown" and not is_airborne and airborne_latch ) {
            // safety 'on_touchdown' means throttle is enabled at start, but is
            // disabled after touchdown (assuming some flying happens)
        } else {
            master_enable = true;
            inceptors_node.setBool("throttle_enable", master_enable);
            event_mgr->add_event("safety", "throttle enabled!");
        }
    } else {
        // throttle enabled, check if we should disable it
        bool make_safe = false;
        if ( safety_mode == "on_ground" and not is_airborne ) {
            make_safe = true;
        } else if ( safety_mode == "on_touchdown" and airborne_latch and not is_airborne ) {
            make_safe = true;
        }
        if ( make_safe ) {
            master_enable = false;
            inceptors_node.setBool("throttle_enable", master_enable);
            event_mgr->add_event("safety", "throttle disabled!");
        }
    }
}