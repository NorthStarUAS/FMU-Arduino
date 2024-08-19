#include <math.h>

#include "../nodes.h"
#include "../comms/events.h"
#include "../util/constants.h"
#include "../util/strutils.h"
#include "../util/wgs84.h"
#include "../util/windtri.h"

#include "route_mgr.h"

static const double sqrt_of_2 = sqrt(2.0);

void route_mgr_t::init() {
    // sanity check, set some conservative values if none are
    // provided in the autopilot config
    if ( config_L1_node.getDouble("bank_limit_deg") < 0.1 ) {
        config_L1_node.setDouble("bank_limit_deg", 25.0);
    }
    if ( config_L1_node.getDouble("period") < 0.1 ) {
        config_L1_node.setDouble("period", 25.0);
    }
    if ( config_L1_node.getDouble("damping") < 0.1 ) {
        config_L1_node.setDouble("damping", 0.7);
    }

    // defaults
    route_node.setString("follow_mode", "leader");
    route_node.setString("start_mode", "first_wpt");
    route_node.setString("completion_mode", "loop");

    // make sure routes start empty
    relative_route.clear();
    abs_route_standby.clear();
    abs_route_active.clear();
}

// build relative route from a property tree (config) node
bool route_mgr_t::build( PropertyNode config_route_node ) {
    relative_route.clear();
    for ( int i = 0; i < config_route_node.getLen("wpt"); i++ ) {
        string child_name = "wpt/" + std::to_string(i);
        PropertyNode child = config_route_node.getChild(child_name.c_str());
        wp_rel_t wp;
        wp.build(child);
        relative_route.push_back(wp);
    }
    printf("loaded %d waypoints.\n", relative_route.size());
    return true;
}

// build a route from a string request
// bool route_mgr_t::build_str( string request ) {
//     vector<string> tokens = split(request, ",");
//     if ( tokens.size() < 3 ) {
//         return false;
//     }
//     abs_route_standby.clear();
//     unsigned int i = 0;
//     while ( i + 4 <= tokens.size() ) {
//         wp_abs_t wp(std::stod(tokens[i+1]), std::stod(tokens[i+2]));
//         abs_route_standby.push_back(wp);
//         i += 4;
//     }
//     printf("Loaded %d waypoints.\n", abs_route_standby.size());
//     return true;
// }

void route_mgr_t::build_start() {
    abs_route_standby.clear();
}

void route_mgr_t::build_append(int32_t lon_raw, int32_t lat_raw) {
    wp_abs_t wp = wp_abs_t(lon_raw, lat_raw);
    abs_route_standby.push_back(wp);
}

// swap active and standby routes
void route_mgr_t::swap() {
    abs_route_active.swap( abs_route_standby );
    use_relative = false;       // switching to absolute route waypoints mode
    current_wp = 0;             // make sure we start at beginning
}

coord_t route_mgr_t::get_wp(unsigned int i) {
    if ( i >= 0 and i < get_active_size() ) {
        if ( use_relative ) {
            wp_rel_t wp = relative_route[i];
            return wp.as_coord();
        } else {
            wp_abs_t wp = abs_route_active[i];
            return wp.as_coord();
        }
    } else {
        return coord_t();
    }
}

coord_t route_mgr_t::get_current_wp() {
    if ( current_wp >= 0 and current_wp < get_active_size() ) {
        if ( use_relative ) {
            wp_rel_t wp = relative_route[current_wp];
            return wp.as_coord();
        } else {
            wp_abs_t wp = abs_route_active[current_wp];
            return wp.as_coord();
        }
    } else {
        return coord_t();
    }
}

coord_t route_mgr_t::get_previous_wp() {
    int prev = current_wp - 1;
    if ( prev < 0 ) {
        prev = get_active_size() - 1;
    }
    if ( prev >= 0 and prev < (int)get_active_size() ) {
        if ( use_relative ) {
            wp_rel_t wp = relative_route[prev];
            return wp.as_coord();
        } else {
            wp_abs_t wp = abs_route_active[prev];
            return wp.as_coord();
        }
    } else {
        return coord_t();
    }
}

void route_mgr_t::increment_wp() {
    if ( current_wp < abs_route_active.size() - 1 ) {
        current_wp += 1;
    } else {
        current_wp = 0;
    }
}

// void route_mgr_t::set_wp( uint16_t i, waypoint_t wp ) {
//     if ( i < active_route.size() ) {
//         active_route[i] = wp;
//     }
// }

// compute one route leg distance per call to spread out expensive
// wgs84 math over multiple frames.
void route_mgr_t::compute_leg_dist( bool reset ) {
    if ( reset ) {
        wp_counter = 0;
        dist_valid = false;
    }

    // compute one route leg distance per call to spread out expensive
    // wgs84 math over multiple frames.
    unsigned int route_size = get_active_size();
    if ( route_size > 0 ) {
        if ( wp_counter >= route_size ) {
            wp_counter = 0;
            dist_valid = true;
        }
        if ( wp_counter < route_size - 1 ) {
            // compute leg course and distance
            coord_t cur;
            coord_t next;
            if ( use_relative ) {
                cur = relative_route[wp_counter].as_coord();
                next = relative_route[wp_counter+1].as_coord();
            } else {
                cur = abs_route_active[wp_counter].as_coord();
                next = abs_route_active[wp_counter+1].as_coord();
            }
            double leg_course;
            double rev_course;
            double leg_dist;
            geo_inverse_wgs_84( cur.lat_deg, cur.lon_deg,
                                next.lat_deg, next.lon_deg,
                                &leg_course, &rev_course, &leg_dist);
            if ( use_relative ) {
                relative_route[wp_counter].leg_dist_m = leg_dist;
            } else {
                abs_route_active[wp_counter].leg_dist_m = leg_dist;
            }
        }
        wp_counter += 1;
    }
}

void route_mgr_t::reposition( bool force ) {
    double home_lon = home_node.getDouble("longitude_deg");
    double home_lat = home_node.getDouble("latitude_deg");
    double home_az = home_node.getDouble("azimuth_deg");

    if ( force or fabs(home_lon - last_lon) > 0.000001 or
         fabs(home_lat - last_lat) > 0.000001 or
         fabs(home_az - last_az) > 0.001 ) {
        for ( unsigned int i = 0; i < relative_route.size(); i++ ) {
            wp_rel_t wp = relative_route[i];
            wp.update_relative_pos(home_lon, home_lat, home_az);
            printf("WPT: %.1f %.1f %lu %lu\n", wp.hdg_deg, wp.dist_m, wp.latitude_raw, wp.longitude_raw);
        }
        if ( comms_node.getBool("display_on") ) {
            printf("ROUTE pattern updated: %.8f %.8f (course = %.1f)\n",
                   home_lon, home_lat, home_az);
        }
        last_lon = home_lon;
        last_lat = home_lat;
        last_az = home_az;
    }
}

float route_mgr_t::get_remaining_distance_from_next_waypoint() {
    float result = 0.0;
    unsigned int route_size = get_active_size();
    for ( unsigned int i = current_wp; i < route_size; i++ ) {
        if ( use_relative ) {
           result += relative_route[i].leg_dist_m;
        } else {
            result += abs_route_active[i].leg_dist_m;
        }
    }
    return result;
}

// Given wind speed, wind direction, and true airspeed (from the
// property tree), as well as a current ground course, and a target
// ground course, compute the estimated true heading (psi, aircraft
// body heading) difference that will take us from the current ground
// course to the target ground course.
//
// Note: this produces accurate tracking, even if the wind estimate is
// wrong.  The primary affect of a poor wind estimate is sub-optimal
// heading error gain.  i.e. the when the current course and target
// course are aligned, this function always produces zero error.
//
// ... and oh by the way, est_cur_hdg_deg and gs1_kt won't exactly
// match truth if the wind estimate has any error at all, but we care
// about the relative heading error, so this function will produce the
// "correct" heading error.
float route_mgr_t::wind_heading_error( float current_crs_deg, float target_crs_deg ) {
    float ws_kt = airdata_node.getDouble("wind_speed_mps") * mps2kt;
    float tas_kt = airdata_node.getDouble("true_airspeed_mps") * mps2kt;
    float wd_deg = airdata_node.getDouble("wind_dir_deg");
    float est_cur_hdg_deg = 0.0;
    float gs1_kt = 0.0;
    float est_nav_hdg_deg = 0.0;
    float gs2_kt = 0.0;
    wind_course( ws_kt, tas_kt, wd_deg, current_crs_deg,
                 &est_cur_hdg_deg, &gs1_kt );
    wind_course( ws_kt, tas_kt, wd_deg, target_crs_deg,
                 &est_nav_hdg_deg, &gs2_kt );
    // print("est cur body:", est_cur_hdg_deg, "est nav body:", est_nav_hdg_deg)
    float hdg_error = 0.0;
    if ( gs1_kt > 0.0 and gs2_kt > 0.0 ) {
        // life is good, we are flying faster than the wind and making progress
        // print " cur:", est_cur_hdg_deg, "gs1:", gs1_kt
        // print " nav:", est_nav_hdg_deg, "gs2:", gs2_kt
        hdg_error = est_cur_hdg_deg - est_nav_hdg_deg;
    } else {
	// Yikes, course cannot be flown, wind too strong!  Compute a
	// heading error relative to the wind "from" direction.  This
	// will cause the aircraft to point it's nose into the wind and
	// kite.  This minimizes a bad situation and gives the operator
	// maximum time to take corrective action.  But hurry and do
	// something!

        // point to next waypoint (probably less good than pointing into
        // the wind.)
        // hdg_error = nav_node.getDouble("heading_deg") - target_crs_deg

        // point to wind (will probably slide laterally due to some
        // inevitable assymetries in bank angle verus turn rate):
        hdg_error = nav_node.getDouble("heading_deg") - wd_deg;
    }
    if ( hdg_error < -180 ) { hdg_error += 360; }
    if ( hdg_error > 180 ) { hdg_error -= 360; }
    // print " body err:", hdg_error

    return hdg_error;
}

void route_mgr_t::update() {
    reposition();  // check if home has changed and reposition if needed

    // string request = route_node.getString("route_request");
    // if ( request.length() ) {
    //     string result = "";
    //     if ( build_str(request) ) {
    //         swap();
    //         reposition(true);
    //         result = "success: " + request;
    //         compute_leg_dist(true);
    //     } else {
    //         result = "failed: " + request;
    //     }
    //     route_node.setString("request_result", result.c_str());
    //     route_node.setString("route_request", "");
    // }

    unsigned int route_size = get_active_size();
    route_node.setInt("route_size", route_size);
    if ( route_size > 0 ) {
        if ( gps_node.getDouble("data_age") < 10.0 ) {
            // track current waypoint of route, (only!) if we have recent gps
            // data

            // route start up logic: if start_mode == first_wpt then there is
            // nothing to do, we simply continue to track wpt 0 if that is the
            // current waypoint.  If start_mode == "first_leg", then if we are
            // tracking wpt 0, increment it so we track the 2nd waypoint along
            // the first leg.  If only a 1 point route is given along with
            // first_leg startup behavior, then don't do that again, force some
            // sort of sane route parameters instead!
            string start_mode = route_node.getString("start_mode");
            if ( start_mode == "first_leg" and current_wp == 0 ) {
                if ( route_size > 1 ) {
                    current_wp += 1;
                } else {
                    route_node.setString("start_mode", "first_wpt");
                    route_node.setString("follow_mode", "direct");
                }
            }

            float L1_period = config_L1_node.getDouble("period");
            float L1_damping = config_L1_node.getDouble("damping");
            float gs_mps = nav_node.getDouble("groundspeed_mps");
            float groundtrack_deg = nav_node.getDouble("groundtrack_deg");
            float tas_mps = airdata_node.getDouble("true_airspeed_mps");

            coord_t prev = get_previous_wp();
            coord_t wp = get_current_wp();
            // event_mgr->add_event("rte", std::to_string((int)(prev.lon_deg*100)) + " " + std::to_string((int)(prev.lat_deg*100)) + " " + std::to_string((int)(wp.lon_deg*100)) + " " + std::to_string((int)(wp.lat_deg*100)));

            // compute direct-to course and distance
            double pos_lon = nav_node.getDouble("longitude_deg");
            double pos_lat = nav_node.getDouble("latitude_deg");
            double direct_course = 0.0;
            double rev_course = 0.0;
            double direct_dist = 0.0;
            geo_inverse_wgs_84( pos_lat, pos_lon, wp.lat_deg, wp.lon_deg,
                                &direct_course, &rev_course, &direct_dist );

            //print pos_lat, pos_lon, ":", wp.lat_deg, wp.lon_deg
            //print " course to:", direct_course, "dist:", direct_dist

            // compute leg course and distance
            double leg_course = 0.0;
            double leg_dist = 0.0;
            geo_inverse_wgs_84( prev.lat_deg, prev.lon_deg,
                                wp.lat_deg, wp.lon_deg,
                                &leg_course, &rev_course, &leg_dist );
            //print prev.lat_deg, prev.lon_deg, " ", wp.lat_deg, wp.lon_deg
            //print " leg course:", leg_course, "dist:", leg_dist

            // difference between ideal (leg) course and direct course
            float angle = leg_course - direct_course;
            if ( angle < -180.0 ) { angle += 360.0; }
            if ( angle > 180.0 ) { angle -= 360.0; }

            // compute cross-track error
            float angle_rad = angle * d2r;
            float xtrack_m = sin(angle_rad) * direct_dist;
            float dist_m = cos(angle_rad) * direct_dist;
            // print("lc: %.1f  dc: %.1f  a: %.1f  xc: %.1f  dd: %.1f" % (leg_course, direct_course, angle, xtrack_m, direct_dist))
            route_node.setDouble( "xtrack_dist_m", xtrack_m );
            route_node.setDouble( "projected_dist_m", dist_m );

            // default distance for waypoint acquisition = direct
            // distance to the target waypoint.  This can be
            // overridden later by leg following and replaced with
            // distance remaining along the leg.
            float nav_dist_m = direct_dist;

            string follow_mode = route_node.getString("follow_mode");
            string completion_mode = route_node.getString("completion_mode");
            float nav_course = 0.0;
            if ( follow_mode == "direct" ) {
                // steer direct to
                nav_course = direct_course;
            } else if ( follow_mode == "leader" ) {
                // scale our L1_dist (something like a target heading
                // gain) proportional to ground speed
                float L1_dist = (1.0 / M_PI) * L1_damping * L1_period * gs_mps;
                float wangle = 0.0;
                if ( L1_dist < 1.0 ) {
                    // ground speed really small or negative (problem?!?)
                    L1_dist = 1.0;
                }
                if ( L1_dist <= fabs(xtrack_m) ) {
                    // beyond L1 distance, steer as directly toward
                    // leg as allowed
                    wangle = 0.0;
                } else {
                    // steer towards imaginary point projected onto
                    // the route leg L1_distance ahead of us
                    wangle = acos(fabs(xtrack_m) / L1_dist) * r2d;
                }
                if ( wangle < 30.0 ) { wangle = 30.0; }
                if ( xtrack_m > 0.0 ) {
                    nav_course = direct_course + angle - 90.0 + wangle;
                } else {
                    nav_course = direct_course + angle + 90.0 - wangle;
                }
                // print("x: %.1f  dc: %.1f  a: %.1f  wa: %.1f  nc: %.1f" % (xtrack_m, direct_course, angle, wangle, nav_course))
                if ( acquired ) {
                    nav_dist_m = dist_m;
                } else {
                    // direct to first waypoint until we've
                    // acquired this route
                    nav_course = direct_course;
                    nav_dist_m = direct_dist;
                }

                // printf("direct=%.1f angle=%.1f nav=%.1f L1=%.1f xtrack=%.1f wangle=%.1f nav_dist=%.1f\n", direct_course, angle, nav_course, L1_dist, xtrack_m, wangle, nav_dist_m)
            }

            float wp_eta_sec = 0.0;
            if ( gs_mps > 0.1 and fabs(nav_dist_m) > 0.1 ) {
                wp_eta_sec = nav_dist_m / gs_mps;
            } else {
                wp_eta_sec = 9999;  // just any sorta big value
            }
            route_node.setDouble( "wp_eta_sec", wp_eta_sec );
            route_node.setDouble( "wp_dist_m", direct_dist );

            if ( nav_course < 0.0 ) { nav_course += 360.0; }
            if ( nav_course > 360.0 ) { nav_course -= 360.0; }
            refs_node.setDouble( "groundtrack_deg", nav_course );

            // allow a crude fudge factor for non-straight airframes or
            // imu mounting errors.  This is essentially the bank angle
            // that yields zero turn rate
            float bank_bias_deg = config_L1_node.getDouble("bank_bias_deg");

            // heading error is computed with wind triangles so this is
            // the actual body heading error, not the ground track
            // error, thus Vomega is computed with tas_mps, not gs_mps
            float omegaA = sqrt_of_2 * M_PI / L1_period;
            //VomegaA = gs_mps * omegaA
            //course_error = nav_node.getDouble("groundtrack_deg") - nav_course
            float VomegaA = tas_mps * omegaA;
            // print "gt:", groundtrack_deg, "nc:", nav_course, "error:", groundtrack_deg - nav_course
            float hdg_error = wind_heading_error(groundtrack_deg, nav_course);
            // clamp to +/-90 so we still get max turn input when flying directly away from the heading.
            if ( hdg_error < -90.0 ) { hdg_error = -90.0; }
            if ( hdg_error > 90.0 ) { hdg_error = 90.0; }
            refs_node.setDouble( "wind_heading_error_deg", hdg_error );

            // target bank angle computed here
            float accel = 2.0 * sin(hdg_error * d2r) * VomegaA;

            float target_bank_deg = -atan(accel / g)*r2d + bank_bias_deg;

            float bank_limit_deg = config_L1_node.getDouble("bank_limit_deg");
            if ( target_bank_deg < -bank_limit_deg + bank_bias_deg ) {
                target_bank_deg = -bank_limit_deg + bank_bias_deg;
            }
            if ( target_bank_deg > bank_limit_deg + bank_bias_deg ) {
                target_bank_deg = bank_limit_deg + bank_bias_deg;
            }
            refs_node.setDouble( "roll_deg", target_bank_deg );

            // estimate distance remaining to completion of route
            if ( dist_valid ) {
                float dist_remaining_m = nav_dist_m +
                    get_remaining_distance_from_next_waypoint();
                route_node.setDouble("dist_remaining_m", dist_remaining_m);
            }
            //if comms_node.getBool("display_on"):
            //    print "next leg: %.1f  to end: %.1f  wpt=%d of %d" % (nav_dist_m, dist_remaining_m, current_wp, len(active_route))

            // logic to mark completion of leg and move to next leg.
            if ( completion_mode == "loop" ) {
                if ( wp_eta_sec < 1.0 ) {
                    acquired = true;
                    increment_wp();
                }
            } else if ( completion_mode == "circle_last_wpt" ) {
                if ( wp_eta_sec < 1.0 ) {
                    acquired = true;
                    if ( current_wp < get_active_size() - 1u ) {
                        increment_wp();
                    } else {
                        wp = get_current_wp();
                        // FIXME: NEED TO GO TO CIRCLE MODE HERE SOME HOW!!!
                        // mission_mgr.start_circle_task(wp.get_target_lon(),
                        //   wp.get_target_lat(),
                        //   0.0, 0.0)
                    }
                }
            } else if ( completion_mode == "extend_last_leg" ) {
                if ( wp_eta_sec < 1.0 ) {
                    acquired = true;
                    if ( current_wp < get_active_size() - 1u ) {
                        increment_wp();
                    } else {
                        // follow the last leg forever
                    }
                }
            }

            // publish current target waypoint
            route_node.setInt("target_waypoint_idx", current_wp);

            // if ( display_on ) {
            // printf("route dist = %0f\n", dist_remaining_m)
            // }
        }
    } else {
        // FIXME: we've been commanded to follow a route, but no
        // route has been defined.

        // We are in ill-defined territory, should we do some sort
        // of circle of our home position?

        // FIXME: need to go to circle mode somehow here!!!!
        // mission_mgr.start_circle_task()
    }

    // compute leg distance for one segment (cycles independent of target waypoint)
    compute_leg_dist();
}

// single global instance of route_mgr
route_mgr_t route_mgr;
