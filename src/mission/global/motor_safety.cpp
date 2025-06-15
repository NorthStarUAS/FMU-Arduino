
#include "../../nodes.h"
#include "../../comms/events.h"
#include "motor_safety.h"

void motor_safety_task_t::init() {
}

void motor_safety_task_t::update() {
    bool is_airborne = environment_node.getBool("is_airborne");
    if ( not airborne_latch and is_airborne ) {
        // aloft
        airborne_latch = true;
    }

    if ( not home_node.getBool("calibrated") and not is_airborne ) {
        // force motor_enable off (override operator switch) as long as home is
        // not calibrated and we are on the ground.  note: a default home
        // position is set when the system comes alive in case we somehow find
        // ourselves in the air, but don't let a normal launch happen until home
        // is properly calibrated.)
        inceptors_node.setBool("motor_enable", false);
    }

    if ( inceptors_node.getBool("motor_enable") ) {
        // if motor is enabled, automatically disable if we have been
        // airborne, but now we are no longer flying (i.e. we landed)
        if ( airborne_latch and not is_airborne ) {
            // touch down
            inceptors_node.setBool("motor_enable", false);
            airborne_latch = false;
            event_mgr->add_event("safety", "landed, motor disabled!");
        }
    }
}