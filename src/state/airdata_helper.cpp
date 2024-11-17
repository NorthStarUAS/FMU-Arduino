// determine if aircraft if airborne or on the ground and time the
// airborne seconds

#include <Arduino.h>

#include "../nodes.h"
#include "../comms/events.h"

#include "airdata_helper.h"

void airdata_helper_t::init() {
    float cruise_kt = PropertyNode("/config/specs").getDouble("cruise_kt");
    if ( cruise_kt > 1.0 ) {
        up_mps = cruise_kt * 0.6 * kt2mps;
        down_mps = cruise_kt * 0.4 * kt2mps;
    }
    airdata_node.setBool("is_airborne", false);
}

void airdata_helper_t::update(float dt) {
    float airdata_agl = airdata_node.getDouble("altitude_agl_m");
    float airspeed_mps = airdata_node.getDouble("airspeed_mps");
    float airdata_alt = airdata_node.getDouble("altitude_m");
    float nav_alt = nav_node.getDouble("altitude_m");

    // determine if aircraft is flying or not
    if ( not is_airborne and airdata_agl >= up_m and airspeed_mps >= up_mps ) {
        // if all conditions over the threshold, we are airborne
        is_airborne = true;
        airdata_node.setBool("is_airborne", true);
        event_mgr->add_event("airdata", "airborne");
    } else if ( is_airborne and airdata_agl <= down_m and airspeed_mps <= down_mps ) {
        // if all conditions under their threshold, we are on the ground
        is_airborne = false;
        airdata_node.setBool("is_airborne", false);
        event_mgr->add_event("airdata", "on ground");
    }

    // compute total time aloft
    if ( is_airborne ) {
        flight_millis += millis() - last_millis;
    }
    airdata_node.setDouble("flight_timer_millis", flight_millis);
    last_millis = millis();

    // compute "true altitude" with baro sensitivity
    if ( nav_node.getInt("status") >= 2 ) {
        float diff = airdata_alt - nav_alt;
        baro_error.update(diff, dt);
    }
    airdata_node.setDouble("altitude_true_m", airdata_alt - baro_error.get_value());
}
