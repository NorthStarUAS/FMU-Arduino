#include "../nodes.h"

#include "ground.h"

// initialize ground estimator variables
void ground_est_t::init() {
    ground_alt_filt.set_time_factor(30.0);
    ground_alt_calibrated = false;
}

void ground_est_t::update(float dt) {
    // determine ground reference altitude.  Average gps altitude over
    // the most recent 30 seconds that we are !is_airborne
    if ( gps_node.getInt("status") >= 3 ) {
        if ( !ground_alt_calibrated ) {
            ground_alt_calibrated = true;
            // subtract out airdata agl in case we somehow we are initing
            // in air (i.e. took off before gps had a good fix?)
            double ground_m = gps_node.getDouble("altitude_m")
                - airdata_node.getDouble("altitude_agl_m");
            ground_alt_filt.init( ground_m );
        }

        if ( !airdata_node.getBool("is_airborne") ) {
            // update ground altitude estimate while not airborne
            // fixme: create a settled (not moving) parameter and use that?
            ground_alt_filt.update( gps_node.getDouble("altitude_m"), dt );
            nav_node.setDouble( "altitude_ground_m", ground_alt_filt.get_value() );
        }
    }
}
