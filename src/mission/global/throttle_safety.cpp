
#include "../../nodes.h"
#include "../../comms/events.h"
#include "throttle_safety.h"

// fixme: I forget exactly how this danced with the pilot transmitter command.
// Let's get the task ported and then figure that out!  Note sensors/switch.cpp
// only updates the switch state when the physical switch moves, so this can
// "force fight" with the transmitter state

void throttle_safety_task_t::init() {
    PropertyNode config_node = PropertyNode("/config/mission/throttle_safety");

    inceptors_node.setBool("throttle_safety", true);
    if ( config_node.hasChild("safety_mode") ) {
        safety_mode = config_node.getString("safety_mode");
    }
}

void throttle_safety_task_t::update() {
    bool is_airborne = airdata_node.getBool("is_airborne");
    if ( not airborne_latch and is_airborne ) {
        airborne_latch = true;
    }

    if ( master_safety ) {
        // safety is on, check if we should remove it (so throttle can run.)
        if ( not gps_node.getBool("settle") ) {
            // do not enable autopilot throttle control if gps hasn't reported a
            // fix and is settled.
        } else if ( safety_mode == "on_ground" and not is_airborne ) {
            // safety "on_ground" means never run throttle unless we are
            // airborne.
        } else if ( safety_mode == "on_touchdown" and not is_airborne and airborne_latch ) {
            // safety 'on_touchdown' means safety starts on, but is
            // switched off after touchdown (assuming some flying happens)
        } else {
            master_safety = false;
            inceptors_node.setBool("throttle_safety", master_safety);
            event_mgr->add_event("safety", "throttle enabled (safety turned off!)");
        }
    } else {
        // safety is off, check if we should turn it on
        bool make_safe = false;
        if ( safety_mode == "on_ground" and not is_airborne ) {
            make_safe = true;
        } else if ( safety_mode == "on_touchdown" and airborne_latch and not is_airborne ) {
            make_safe = true;
        }
        if ( make_safe ) {
            master_safety = true;
            inceptors_node.setBool("throttle_safety", master_safety);
            event_mgr->add_event("safety", "throttle disabled (safety turned on!)");
        }
    }
}