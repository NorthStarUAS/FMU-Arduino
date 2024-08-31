#pragma once

#include <vector>
using std::vector;

#include "../../props2.h"

#include "waypoint.h"

class route_mgr_t {

public:

    void init();
    bool build( PropertyNode config_route_node );
    // bool build_str( string request );
    void build_start();
    void build_append(int32_t lon_raw, int32_t lat_raw);
    void swap();
    uint16_t get_active_size() {
        if ( use_relative ) {
            return relative_route.size();
        } else {
            return abs_route_active.size();
        }
    }
    // void set_active_size( uint16_t n ) { active_route.resize(n); }
    coord_t get_wp(unsigned int i);
    coord_t get_current_wp();
    coord_t get_previous_wp();
    void increment_wp();
    // void set_wp( uint16_t i, waypoint_t wp );
    void compute_leg_dist( bool reset=false );
    void reposition( bool force=false );
    float get_remaining_distance_from_next_waypoint();
    float wind_heading_error( float current_crs_deg, float target_crs_deg );
    void update();

private:

    bool use_relative = true;
    vector<wp_rel_t> relative_route;
    vector<wp_abs_t> abs_route_active;
    vector<wp_abs_t> abs_route_standby;

    uint16_t current_wp = 0;
    bool acquired = false;

    double last_lon = 0.0;
    double last_lat = 0.0;
    float last_az = 0.0;
    uint16_t wp_counter = 0;    // for dribble to gcs
    bool dist_valid = false;
};
