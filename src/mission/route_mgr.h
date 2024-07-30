#include <vector>
using std::vector;

#include "../props2.h"

#include "waypoint.h"

class route_mgr_t {

public:

    void init();
    bool build( PropertyNode config_route_node );
    bool build_str( string request );
    void swap();
    int get_active_size() { return active_route.size(); }
    void set_active_size( uint16_t n ) { active_route.resize(n); }
    waypoint_t get_current_wp();
    waypoint_t get_previous_wp();
    void increment_wp();
    void set_wp( uint16_t i, waypoint_t wp );
    void dribble( bool reset=false );
    void reposition( bool force=false );
    float get_remaining_distance_from_next_waypoint();
    float wind_heading_error( float current_crs_deg, float target_crs_deg );
    void update();

private:

    vector<waypoint_t> active_route;
    vector<waypoint_t> standby_route;
    unsigned int current_wp = 0;
    bool acquired = false;

    double last_lon = 0.0;
    double last_lat = 0.0;
    float last_az = 0.0;
    uint16_t wp_counter = 0;    // for dribble
    bool dist_valid = false;
};

extern route_mgr_t route_mgr;
