#include "../nodes.h"

#include "ground.h"

// initialize ground estimator variables
void ground_est_t::init() {
    airdata_ground_alt.set_time_factor(30.0);
    nav_ground_alt.set_time_factor(30.0);
}

void ground_est_t::update(float dt) {
    // determine ground reference altitude.  Average baro and nav altitude over
    // the most recent 30 seconds that we are !is_airborne

    // note there is a chance of being a bit self referential here ... ground
    // altitude informs is_airborne and is_airborne informs ground altitude.  It
    // will help to follow a careful procedure of starting up and calibrating on
    // the ground, versus starting up in the air!

    // make sure airdata_ground_alt is inited to something even if we manage to
    // startup with is_airborne = true
    if ( not ground_inited ) {
        ground_inited = true;
        airdata_ground_alt.init( airdata_node.getDouble("altitude_m") );
    }

    // update ground altitude estimates while not airborne
    // fixme: create a settled (not moving) parameter and use that?
    if ( not airdata_node.getBool("is_airborne") ) {
        airdata_ground_alt.update( airdata_node.getDouble("altitude_m"), dt );
        // ok it's a bit weird, but we just save agl here, we don't need to
        // publish the baro ground altitude, just the difference
    }
    airdata_node.setDouble("altitude_agl_m", airdata_node.getDouble("altitude_m") - airdata_ground_alt.get_value());

    if ( nav_node.getInt("status") >= 2 ) {
        if ( not airdata_node.getBool("is_airborne") ) {
            nav_ground_alt.update( nav_node.getDouble("altitude_m"), dt );
            // ok it's a bit weird, but we save 'abs' ground altitude computed
            // from the nav solution in the airdata node
            airdata_node.setDouble( "altitude_ground_m", nav_ground_alt.get_value() );
        }
    }
}
