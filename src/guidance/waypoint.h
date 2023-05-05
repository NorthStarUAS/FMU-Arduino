#include "../props2.h"

class waypoint_t {

public:
    bool absolute = true;
    double lon_deg = 0.0;
    double lat_deg = 0.0;
    float hdg_deg = 0.0;
    float dist_m = 0.0;
    float leg_dist_m = 0.0;

    waypoint_t() {}
    waypoint_t( int mode, double coord1, double coord2 );
    void build( PropertyNode config_node );
    void update_relative_pos( double home_lon_deg, double home_lat_deg,
                              float ref_heading_deg );
};
