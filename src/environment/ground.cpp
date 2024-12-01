#include "../nodes.h"

#include "ground.h"

// initialize ground estimator variables
void ground_est_t::init() {
    airdata_ground_alt.set_time_factor(30.0);
    gps_ground_alt.set_time_factor(30.0);
}

void ground_est_t::update(float dt) {
    // determine ground reference altitude.  Average baro and nav altitude over
    // the most recent 30 seconds that we are !is_airborne

    // note there is a chance of being a bit self referential here ... ground
    // altitude informs is_airborne and is_airborne informs ground altitude.  It
    // will help to follow a careful procedure of starting up and calibrating on
    // the ground, versus starting up in the air!
    float airdata_altitude_m = airdata_node.getDouble("altitude_m");

    // make sure airdata_ground_alt is inited to something even if we manage to
    // startup with is_airborne = true (but also make sure the airdata node has
    // hopefully some sensible data.)
    if ( not ground_inited and airdata_node.getUInt("millis") > 1000 ) {
        airdata_ground_alt.init( airdata_altitude_m );
        ground_inited = true;
    }

    // update ground altitude estimates while not airborne
    // fixme: create a settled (not moving) parameter and use that?
    if ( not environment_node.getBool("is_airborne") ) {
        airdata_ground_alt.update( airdata_altitude_m, dt );
        // ok it's a bit weird, but we just save agl here, we don't need to
        // publish the baro ground altitude, just the difference
    }
    environment_node.setDouble("altitude_agl_m", airdata_altitude_m - airdata_ground_alt.get_value());

    if ( gps_node.getInt("status") >= 3 ) {
        if ( not environment_node.getBool("is_airborne") ) {
            gps_ground_alt.update( gps_node.getDouble("altitude_m"), dt );
            // publish 'gps' ground altitude average
            environment_node.setDouble( "altitude_ground_m", gps_ground_alt.get_value() );
        }
    }
}
