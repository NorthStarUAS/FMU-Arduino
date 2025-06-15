// home_mgr: set the home location to the current location as soon as there is
// valid information to do this.  Compute distance, heading, and x, y components
// to home.

#include <math.h>

#include "../../nodes.h"
#include "../../util/constants.h"
#include "../../util/wgs84.h"

#include "home_mgr.h"

void home_mgr_t::init() {
    home_node.setBool("valid", false);
    home_node.setBool("calibrated", false);
}

void home_mgr_t::update() {
    if ( not home_node.getBool("valid") ) {
        if ( gps_node.getDouble("gps_age") < 1.0 and gps_node.getBool("settle") ) {
            // Save current position as startup position
            startup_node.setDouble("longitude_deg", gps_node.getDouble("longitude_deg"));
            startup_node.setDouble("latitude_deg", gps_node.getDouble("latitude_deg"));
            startup_node.setDouble("altitude_m", gps_node.getDouble("altitude_m"));
            startup_node.setBool("valid", true);

            // Set initial "home" position.
            home_node.setDouble("longitude_deg", gps_node.getDouble("longitude_deg"));
            home_node.setDouble("latitude_deg", gps_node.getDouble("latitude_deg"));
            home_node.setDouble("altitude_m", gps_node.getDouble("altitude_m"));
            home_node.setDouble("azimuth_deg", 0.0);
            home_node.setBool("valid", true);
        }
    } else {
        // compute a mini cartesian (2d) system relative to home in meters

        // FIXME?: the parametric task is the only thing that uses this so why
        // not put this code over into the parametric.py module?  Or ... should
        // more things use x, y local coordinates?

        double course_deg, rev_deg, dist_m;
        geo_inverse_wgs_84( nav_node.getDouble("latitude_deg"),
                            nav_node.getDouble("longitude_deg"),
                            home_node.getDouble("latitude_deg"),
                            home_node.getDouble("longitude_deg"),
                            &course_deg, &rev_deg, &dist_m );
        home_node.setDouble("course_deg", course_deg);
        home_node.setDouble("dist_m", dist_m);

        float theta = rev_deg * d2r;
        float x = sin(theta) * dist_m;
        float y = cos(theta) * dist_m;
        home_node.setDouble("x_m", x);
        home_node.setDouble("y_m", y);
    }
}