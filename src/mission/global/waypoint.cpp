#include "../../util/wgs84.h"

#include "waypoint.h"

void wp_rel_t::build( PropertyNode config_waypoint_node ) {
    if ( config_waypoint_node.hasChild("lon_deg") and config_waypoint_node.hasChild("lat_deg") ) {
        double lon_deg = config_waypoint_node.getDouble("lon_deg");
        double lat_deg = config_waypoint_node.getDouble("lat_deg");
        printf("Absolute wp in a relative route: %.8f %.8f\n", lon_deg, lat_deg);
    } else if ( config_waypoint_node.hasChild("heading_deg") and config_waypoint_node.hasChild("dist_m") ) {
        hdg_deg = config_waypoint_node.getDouble("heading_deg");
        dist_m = config_waypoint_node.getDouble("dist_m");
        printf("WPT: %4.0f deg %.0f m\n", hdg_deg, dist_m);
    } else {
        printf("Error in route waypoint config_waypoint_node logic:\n");
        config_waypoint_node.pretty_print();
    }
}

void wp_rel_t::update_relative_pos( double home_lon_deg, double home_lat_deg, float ref_heading_deg ) {
    float course = ref_heading_deg + hdg_deg;
    if ( course < 0.0 ) { course += 360.0; }
    if ( course > 360.0 ) { course -= 360.0; }
    double az2;
    double lon_deg;
    double lat_deg;
    geo_direct_wgs_84( home_lat_deg, home_lon_deg, course, dist_m, &lat_deg, &lon_deg, &az2 );
    longitude_raw = lon_deg * 10000000;
    latitude_raw = lat_deg * 10000000;
}

void wp_abs_t::build( PropertyNode config_waypoint_node ) {
    if ( config_waypoint_node.hasChild("lon_deg") and config_waypoint_node.hasChild("lat_deg") ) {
        longitude_raw = config_waypoint_node.getDouble("lon_deg") * 10000000;
        latitude_raw = config_waypoint_node.getDouble("lat_deg") * 10000000;
        printf("WPT: %lu %lu\n", longitude_raw, latitude_raw);
    } else if ( config_waypoint_node.hasChild("heading_deg") and config_waypoint_node.hasChild("dist_m") ) {
        float hdg_deg = config_waypoint_node.getDouble("heading_deg");
        float dist_m = config_waypoint_node.getDouble("dist_m");
        printf("Relative waypoint in an absolute route: %4.0f deg %.0f m\n", hdg_deg, dist_m);
    } else {
        printf("Error in route waypoint config_waypoint_node logic:\n");
        config_waypoint_node.pretty_print();
    }
}
