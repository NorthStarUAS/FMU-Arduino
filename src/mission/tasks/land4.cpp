#include <string>
using std::string;

#include "../../nodes.h"
#include "../../comms/events.h"
#include "../../fcs/fcs_mgr.h"
#include "../../util/constants.h"
#include "../../util/wgs84.h"
#include "../mission_mgr.h"
#include "land4.h"

land_task4_t::land_task4_t() {
    name = "land";
}

void land_task4_t::activate() {
    active = true;

    if ( not environment_node.getBool("is_airborne") ) {
        // not airborne, nothing to do, fixme: we want to immediately complete?
        return;
    }

    // build the approach with the current configuration
    PropertyNode config_node = PropertyNode("/config/mission/land");
    build_approach(config_node);

    fcs_mgr->set_mode("basic+tecs");
    config_tecs_node.setDouble("weight_bal", 2.0);  // 0 - 2, val==2 uses only alt error for enery diff (pitch), power controls total energy still
    mission_node.setString("mode", "circle");
    refs_node.setDouble("airspeed_kt", config_node.getDouble("approach_speed_kt"));
    refs_node.setDouble("flaps_setpoint", config_node.getDouble("flaps"));

    // start at the beginning of the route (in case we inherit a partially flown
    // approach from earlier in the flight) approach_mgr.restart() // FIXME
    circle_capture = false;
    gs_capture = false;
    in_flare = false;
}

void land_task4_t::build_approach(PropertyNode config_node) {
    // Setup a descending circle tangent to the final approach path.  The
    // touchdown point is 'home' and the final heading is home azimuth.  The
    // final approach route is simply two points.  A circle decent if flown
    // until the glideslope is captured and then at the correct exit point the
    // task switches to route following mode.  Altitude/decent is managed by
    // this task.

    // fetch config parameters
    if ( config_node.hasChild("transit_alt_agl_ft") ) {
        transit_alt_agl_ft = config_node.getDouble("transit_alt_agl_ft");
        if ( transit_alt_agl_ft < 100.0 ) {
            transit_alt_agl_ft = 100.0;
        }
    }
    circle_radius_m = config_node.getDouble("circle_radius_m");
    if ( circle_radius_m < 50.0 ) {
        circle_radius_m = 50.0;
    }
    final_leg_m = config_node.getDouble("final_leg_m");
    float lateral_offset_m = config_node.getDouble("lateral_offset_m");
    string dir = config_node.getString("direction");
    if ( dir == "left" ) {
        side = -1.0;
    } else if ( dir == "right" ) {
        side = 1.0;
    }
    final_heading_deg = home_node.getDouble("azimuth_deg");
    float glideslope_deg = config_node.getDouble("glideslope_deg");
    if ( glideslope_deg < 3.0 ) {
        glideslope_deg = 3.0;
    }
    alt_base_agl_ft = config_node.getDouble("alt_base_agl_ft");
    flare_pitch_deg = config_node.getDouble("flare_pitch_deg");
    flare_seconds = config_node.getDouble("flare_seconds");
    if ( flare_seconds < 1.0 ) {
        flare_seconds = 1.0;
    }

    // // final leg length: compute horizontal distance to 175' at the
    // // configured glideslope angle
    // float min_alt = 175;
    tan_gs = tan(glideslope_deg*d2r);
    // if ( tan_gs > 0 ) {
    //     float hdist_m = (min_alt / tan_gs) * ft2m;
    //     // printf("hdist_m: %.1f\n", hdist_m);
    //     float half_circle_m = circle_radius_m * M_PI;
    //     // printf("half_circle_m: %.1f\n", half_circle_m);
    //     final_leg_m = hdist_m - half_circle_m + extend_final_leg_m;
    //     if ( final_leg_m < 50 ) {
    //         final_leg_m = 50;
    //     }
    // } else {
    //     final_leg_m = 4.0 * circle_radius_m + extend_final_leg_m;
    // }
    printf("final_leg_m: %.1f\n", final_leg_m);

    // touchdown point
    float sign = 1;
    if ( lateral_offset_m < 0 ) {
        sign = -1;
    }
    float hdg = fmod(final_heading_deg + 90 * sign, 360.0);
    double tgt_lat, tgt_lon, az2;
    geo_direct_wgs_84( home_node.getDouble("latitude_deg"), home_node.getDouble("longitude_deg"), hdg, abs(lateral_offset_m),
                       &tgt_lat, &tgt_lon, &az2);

    // tangent point (reuse hdg, az2)
    hdg = fmod(final_heading_deg + 180,  360);
    double tan_lat, tan_lon;
    geo_direct_wgs_84( tgt_lat, tgt_lon, hdg, final_leg_m, &tan_lat, &tan_lon, &az2);

    // circle center (reuse hdg, az2)
    hdg = fmod(final_heading_deg + side * 90, 360);
    double cc_lat, cc_lon;
    geo_direct_wgs_84( tan_lat, tan_lon, hdg, circle_radius_m, &cc_lat, &cc_lon, &az2);

    // configure circle task
    circle_node.setDouble("latitude_deg", cc_lat);
    circle_node.setDouble("longitude_deg", cc_lon);
    circle_node.setString("direction", dir);
    circle_node.setDouble("radius_m", circle_radius_m);

    // create and request approach route
    mission_mgr->route_mgr.build_start();
    // start of final leg point
    mission_mgr->route_mgr.build_append(tan_lon*10000000, tan_lat*10000000);
    // touchdown target point
    mission_mgr->route_mgr.build_append(tgt_lon*10000000, tgt_lat*10000000);
    // set route modes and activate
    route_node.setString("start_mode", "first_wpt");
    route_node.setString("follow_mode", "leader");
    route_node.setString("completion_mode", "extend_last_leg");
    mission_mgr->route_mgr.swap();                  // make standby route active
    mission_mgr->route_mgr.compute_leg_dist(true);  // start leg distance calculating from the beginning

    // seed route dist_remaining_m value so it is not zero or left
    // over from previous route.
    route_node.setDouble("dist_remaining_m", final_leg_m);
}

void land_task4_t::update(float dt) {
    if ( not active ) {
        return;
    }

    // add ability for pilot to bias the glideslope altitude using
    // stick/elevator (negative elevator is up.)
    float alt_bias_ft = alt_base_agl_ft - inceptors_node.getDouble("pitch") * 25.0;

    // compute minimum 'safe' altitude (replaced by explicit transit_alt_agl_ft)
    // float safe_dist_m = M_PI * circle_radius_m + final_leg_m;
    // float safe_alt_ft = safe_dist_m * tan_gs * m2ft + alt_bias_ft;

    float circle_pos = 0.0;
    string mode = mission_node.getString("mode");
    if ( mode == "circle" ) {
        // circle descent portion of the approach
        double pos_lon = nav_node.getDouble("longitude_deg");
        double pos_lat = nav_node.getDouble("latitude_deg");
        double center_lon = circle_node.getDouble("longitude_deg");
        double center_lat = circle_node.getDouble("latitude_deg");
        // compute course and distance to center of target circle
        double course_deg, rev_deg, cur_dist_m;
        geo_inverse_wgs_84( center_lat, center_lon, pos_lat, pos_lon, &course_deg, &rev_deg, &cur_dist_m);
        // test for circle capture
        if ( not circle_capture ) {
            float fraction = abs(cur_dist_m / circle_radius_m);
            // printf("heading to circle: %.1f %.1f", err, fraction);
            if ( fraction > 0.80 and fraction < 1.20 ) {
                // within 20% of target circle radius, call the circle capture
                event_mgr->add_event("land", "descent circle capture");
                circle_capture = true;
            }
        }

        // compute portion of circle remaining to tangent point
        float current_crs = course_deg + side * 90;
        if ( current_crs > 360.0 ) { current_crs -= 360.0; }
        if ( current_crs < 0.0 ) { current_crs += 360.0; }
        circle_pos = (final_heading_deg - current_crs) * side;  // position on circle descent
        if ( circle_pos < -180.0 ) { circle_pos += 360.0; }
        if ( circle_pos > 180.0 ) { circle_pos -= 360.0; }
        // printf("circle_pos: %.1f, %.1f, %.1f %.1f\n", nav_node.getDouble("groundtrack_deg"), current_crs, final_heading_deg, circle_pos);
        float angle_rem_rad = M_PI;
        if ( circle_capture and gs_capture and circle_pos > -10 ) {
            // circling, captured circle, captured glideslope and within 180
            // degrees towards tangent point (or just slightly passed).
            // Otherwise hold minimum of final_leg_m + M_PI * circle_radius for
            // dist remaining until glideslope acquired
            angle_rem_rad = circle_pos * d2r;
        }
        // distance to edge of circle + remaining circumference of circle +
        // final approach leg
        dist_rem_m = (cur_dist_m - circle_radius_m) + angle_rem_rad * circle_radius_m + final_leg_m;
        // printf("circle: %.1f %.1f %.1f %.1f", dist_rem_m, circle_radius_m, final_leg_m, cur_dist_m);
        if ( circle_capture and gs_capture ) {
            // we are on the circle and on the glide slope, lets look for our
            // lateral exit point
            if ( fabs(circle_pos) <= 5.0 ) {
                event_mgr->add_event("land", "transition to final");
                mission_node.setString("mode", "route");
            }
        }
    } else {
        // on final approach
        if ( route_node.getBool("dist_valid") ) {
            dist_rem_m = route_node.getDouble("dist_remaining_m");
        }
    }

    // compute glideslope/target elevation
    float alt_m = dist_rem_m * tan_gs;
    // printf(" %s dist = %.1f alt = %.1f\n", mode, dist_rem_m, alt_m);

    // Compute target altitude.
    float cur_alt = environment_node.getDouble("altitude_agl_m") * m2ft;
    float cur_ref_alt = refs_node.getDouble("altitude_agl_ft");
    float new_ref_alt = alt_m * m2ft + alt_bias_ft;

    // We want to avoid wasting energy needlessly gaining altitude. Once the
    // landing task has started, don't climb above the current ref altitude ...
    if ( new_ref_alt > cur_ref_alt ) {
        new_ref_alt = cur_ref_alt;
    }

    // ... unless prior to descent circle capture, never allow target altitude
    // lower than transit altitude
    if ( not circle_capture ) {
        // printf("safe: %.1f new: %.1f\n", safe_alt_ft, new_ref_alt);
        if ( new_ref_alt < transit_alt_agl_ft ) {
            new_ref_alt = transit_alt_agl_ft;
        }
    }

    refs_node.setDouble("altitude_agl_ft", new_ref_alt);

    // compute error metrics relative to ideal glide slope
    float alt_error_ft = cur_alt - (alt_m * m2ft + alt_bias_ft);
    float gs_error = atan2(alt_error_ft * ft2m, dist_rem_m) * r2d;
    // printf("alt_error_ft = %.1f  gs err = %.1f\n", alt_error_ft, gs_error);

    if ( circle_capture and not gs_capture ) {
        // on the circle, but haven't intercepted gs
        // printf()"waiting for gs intercept\n");
        if ( gs_error <= 1.0 and circle_pos >= 0 ) {
            // 1 degree or less glide slope error and on the 2nd half of the
            // circle, call the gs captured
            event_mgr->add_event("land", "glide slope capture");
            gs_capture = true;
        }
    }

    // compute time to touchdown at current ground speed (assuming the
    // navigation system has lined us up properly
    float ground_speed_ms = nav_node.getDouble("groundspeed_mps");
    float seconds_to_touchdown = 0;
    if ( ground_speed_ms > 0.01 ) {
        seconds_to_touchdown = dist_rem_m / ground_speed_ms;
    } else {
        seconds_to_touchdown = 1000.0; // lots
    }
    // printf()"dist_rem_m = %.1f gs = %.1f secs = %.1f", dist_rem_m, ground_speed_ms, seconds_to_touchdown);

    if ( mode == "route" and seconds_to_touchdown <= flare_seconds and not in_flare ) {
        // within x seconds of touchdown horizontally and in route mode (final
        // approach leg).  Note these are padded numbers because we don't know
        // the truth exactly ... we could easily be closer or lower or further
        // or higher.  Our flare strategy is to smoothly pull throttle to idle,
        // while smoothly pitching to the target flare pitch (as configured in
        // the task definition.)
        event_mgr->add_event("land", "start flare");
        in_flare = true;
        flare_start_time = imu_node.getUInt("millis") / 1000.0;
        approach_power = outputs_node.getDouble("power");
        approach_pitch = refs_node.getDouble("pitch_deg");
        flare_pitch_range = approach_pitch - flare_pitch_deg;
        fcs_mgr->set_mode("basic");
    }

    if ( in_flare ) {
        float elapsed = imu_node.getUInt("millis") / 1000.0 - flare_start_time;
        float percent = elapsed / flare_seconds;  // earlier we forced flare_seconds to be at least 1.0 so this is a safe divide
        if ( percent > 1.0 ) {
            percent = 1.0;
        }
        refs_node.setDouble("pitch_deg", approach_pitch - percent * flare_pitch_range);
        outputs_node.setDouble("power", approach_power * (1.0 - percent));
        // printf("FLARE: elapsed=%.1f percent=%.2f speed=%.1f throttle=%.1f\n",
        //       elapsed, percent,
        //       approach_speed_kt - percent * flare_pitch_range,
        //       approach_throttle * (1.0 - percent))

        if ( alt_base_agl_ft >= 50 and elapsed > flare_seconds ) {
            // special logic for landing practice/demo: we are at the end of the
            // flare procedure, but have specified an alt_base raising the
            // landing pattern above ground, so immediate transition to the
            // launch task (not tested, but if we are somehow on the ground now,
            // motor enable should be false and we won't be able to actually
            // launch)
            mission_node.setString("request", "launch");
        }
    }

    // if ( display_on ) {
    //    printf("land dist = %.0f target alt = %.0f\n",
    //           dist_rem_m, alt_m * SG_METER_TO_FEET + alt_bias_ft)
}

bool land_task4_t::is_complete() {
    if ( not active ) {
        // not active == complete
        return true;
    } else if ( not environment_node.getBool("is_airborne") ) {
        return true;
    }
    return false;
}

void land_task4_t::close() {
    refs_node.setDouble("flaps_setpoint", 0.0);  // raise flaps
    active = false;
}