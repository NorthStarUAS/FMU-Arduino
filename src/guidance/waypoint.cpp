#include "../util/wgs84.h"

#include "waypoint.h"

waypoint_t::waypoint_t( int mode, double coord1, double coord2 ) {
    if ( mode == 0 ) {
        absolute = false;
        dist_m = coord1;
        hdg_deg = coord2;
    } else {
        absolute = true;
        lon_deg = coord1;
        lat_deg = coord2;
    }
}

void waypoint_t::build( PropertyNode config_node ) {
    if ( config_node.hasChild("lon_deg") and
         config_node.hasChild("lat_deg") ) {
        lon_deg = config_node.getDouble("lon_deg");
        lat_deg = config_node.getDouble("lat_deg");
        absolute = true;
        printf("WPT: %.8f %.8f\n", lon_deg, lat_deg);
    } else if ( config_node.hasChild("heading_deg") and
                config_node.hasChild("dist_m") ) {
        hdg_deg = config_node.getDouble("heading_deg");
        dist_m = config_node.getDouble("dist_m");
        absolute = false;
        printf("WPT: %4.0f deg %.0f m\n", hdg_deg, dist_m);
    } else {
        printf("Error in route waypoint config_node logic:\n");
        config_node.pretty_print();
    }
}

void waypoint_t::update_relative_pos( double home_lon_deg, double home_lat_deg,
                                      float ref_heading_deg ) {
    if ( !absolute ) {
        float course = ref_heading_deg + hdg_deg;
        if ( course < 0.0 ) { course += 360.0; }
        if ( course > 360.0 ) { course -= 360.0; }
        double az2;
        geo_direct_wgs_84( home_lat_deg, home_lon_deg, course, dist_m,
                           &lat_deg, &lon_deg, &az2 );
    } else {
        printf("Error: cannot update relative position of absolute waypoint\n");
    }
}
