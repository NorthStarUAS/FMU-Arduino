#include "../props2.h"

struct coord_t {
    double lon_deg = 0.0;
    double lat_deg = 0.0;
};

class wp_rel_t {

public:

    float dist_m = 0.0;
    float hdg_deg = 0.0;
    uint32_t longitude_raw;
    uint32_t latitude_raw;
    float leg_dist_m = 0.0;

    wp_rel_t() {}
    wp_rel_t( float dist, float hdg ) {
        dist_m = dist;
        hdg_deg = hdg;
    }
    ~wp_rel_t() {}

    void build( PropertyNode config_waypoint_node );
    void update_relative_pos( double home_lon_deg, double home_lat_deg,
                              float ref_heading_deg );
    coord_t as_coord() {
        coord_t coord;
        coord.lon_deg = longitude_raw / 10000000.0;
        coord.lat_deg = latitude_raw / 10000000.0;
        return coord;
    }
};

class wp_abs_t {

public:

    uint32_t longitude_raw;
    uint32_t latitude_raw;
    float leg_dist_m = 0.0;

    wp_abs_t() {}
    wp_abs_t( uint32_t lon_raw, uint32_t lat_raw ) {
        longitude_raw = lon_raw;
        latitude_raw = lat_raw;
    }
    ~wp_abs_t() {}

    void build( PropertyNode config_waypoint_node );
    coord_t as_coord() {
        coord_t coord;
        coord.lon_deg = longitude_raw / 10000000.0;
        coord.lat_deg = latitude_raw / 10000000.0;
        return coord;
    }
};
