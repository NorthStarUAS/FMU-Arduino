#include <math.h>

#include "../../nodes.h"
#include "../../util/constants.h"
#include "../../util/wgs84.h"

#include "circle_mgr.h"

static const double sqrt_of_2 = sqrt(2.0);

void circle_mgr_t::init() {
    // sanity check, set some conservative values if none are provided
    // in the autopilot config
    if ( config_L1_node.getDouble("bank_limit_deg") < 0.1 ) {
        config_L1_node.setDouble("bank_limit_deg", 25.0);
    }
    if ( config_L1_node.getDouble("period") < 0.1 ) {
        config_L1_node.setDouble("period", 25.0);
    }
}

void circle_mgr_t::update() {
    string direction_str = circle_node.getString("direction");
    float direction = 1.0;
    if ( direction_str == "right" ) {
        direction = -1.0;
    } else if ( direction_str == "left" ) {
        direction = 1.0;
    } else {
        circle_node.setString("direction", "left");
        direction = 1.0;
    }

    double lon_deg = nav_node.getDouble("longitude_deg");
    double lat_deg = nav_node.getDouble("latitude_deg");
    if ( fabs(lon_deg) < 0.1 and fabs(lat_deg) < 0.1 ) {
        // no valid current position, bail out.
        return;
    }

    double center_lon = 0.0;
    double center_lat = 0.0;
    if ( circle_node.hasChild("longitude_deg") and circle_node.hasChild("latitude_deg") ) {
        // we have a valid circle center
        center_lon = circle_node.getDouble("longitude_deg");
        center_lat = circle_node.getDouble("latitude_deg");
    } else {
        // we have a valid position, but no valid circle center, use
        // current position.  (sanity fallback)
        circle_node.setDouble("longitude_deg", lon_deg);
        circle_node.setDouble("latitude_deg", lat_deg);
        circle_node.setDouble("radius_m", 100.0);
        circle_node.setString("direction", "left");
        center_lon = lon_deg;
        center_lat = lat_deg;
    }

    // compute course and distance to center of target circle
    // fixme: should reverse this and direction sense to match 'land.py' and make more sense
    double course_deg, rev_deg, dist_m;
    geo_inverse_wgs_84( lat_deg, lon_deg, center_lat, center_lon,
                        &course_deg, &rev_deg, &dist_m );

    // compute ideal ground course to be on the circle perimeter if at
    // ideal radius
    float ideal_crs = course_deg + direction * 90;
    if ( ideal_crs > 360.0 ) { ideal_crs -= 360.0; }
    if ( ideal_crs < 0.0 ) { ideal_crs += 360.0; }

    // (in)sanity check
    float radius_m = 0.0;
    if ( circle_node.hasChild("radius_m") ) {
        radius_m = circle_node.getDouble("radius_m");
        if ( radius_m < 35 ) { radius_m = 35; }
    } else {
        radius_m = 100;
        circle_node.setDouble("radius_m", radius_m);
    }

    // compute a target ground course based on our actual radius distance
    float target_crs = ideal_crs;
    if ( dist_m < radius_m ) {
        // inside circle, adjust target heading to expand our circling
        // radius
        float offset_deg = direction * 90.0 * (1.0 - dist_m / radius_m);
        target_crs += offset_deg;
    } else if ( dist_m > radius_m ) {
        // outside circle, adjust target heading to tighten our
        // circling radius
        float offset_dist = dist_m - radius_m;
        if ( offset_dist > radius_m ) { offset_dist = radius_m; }
        float offset_deg = direction * 90 * offset_dist / radius_m;
        target_crs -= offset_deg;
    }
    if ( target_crs > 360.0 ) { target_crs -= 360.0; }
    if ( target_crs < 0.0 ) { target_crs += 360.0; }
    refs_node.setDouble( "groundtrack_deg", target_crs );

    // L1 'mathematical' response to error
    float L1_period = config_L1_node.getDouble("period");  // gain
    float gs_mps = nav_node.getDouble("groundspeed_mps");
    float omegaA = sqrt_of_2 * M_PI / L1_period;
    float VomegaA = gs_mps * omegaA;
    float course_error = nav_node.getDouble("groundtrack_deg") - target_crs;

    // wrap to +/- 180
    if ( course_error < -180.0 ) { course_error += 360.0; }
    if ( course_error >  180.0 ) { course_error -= 360.0; }
    // clamp to +/-90
    if ( course_error < -90.0 ) { course_error = -90.0; }
    if ( course_error > 90.0 ) { course_error = 90.0; }
    refs_node.setDouble( "course_error_deg", course_error );

    // accel: is the lateral acceleration we need to compensate for
    // heading error
    float accel = 2.0 * sin(course_error * d2r) * VomegaA;

    // circling acceleration needed for our current distance from center
    float turn_accel = 0.0;
    if ( dist_m > 0.1 ) {
        turn_accel = direction * gs_mps * gs_mps / dist_m;
    }

    // allow a crude fudge factor for non-straight airframes or imu
    // mounting errors.  This is essentially the bank angle that
    // yields zero turn rate
    float bank_bias_deg = config_L1_node.getDouble("bank_bias_deg");

    // compute desired acceleration = acceleration required for course
    // correction + acceleration required to maintain turn at current
    // distance from center.
    float total_accel = accel + turn_accel;

    float target_bank = -atan( total_accel / g );
    float target_bank_deg = target_bank * r2d + bank_bias_deg;

    float bank_limit_deg = config_L1_node.getDouble("bank_limit_deg");
    if ( target_bank_deg < -bank_limit_deg + bank_bias_deg) {
        target_bank_deg = -bank_limit_deg + bank_bias_deg;
    }
    if ( target_bank_deg > bank_limit_deg + bank_bias_deg ) {
        target_bank_deg = bank_limit_deg + bank_bias_deg;
    }

    refs_node.setDouble( "roll_deg", target_bank_deg );

    route_node.setDouble( "wp_dist_m", dist_m );
    if ( gs_mps > 0.1 ) {
        route_node.setDouble( "wp_eta_sec", dist_m / gs_mps );
    } else {
        route_node.setDouble( "wp_eta_sec", 0.0 );
    }
}
