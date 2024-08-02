#include <vector>
using std::vector;

#include "../nodes.h"
#include "state.h"

MissionState::MissionState(bool save_modes, bool save_circle, bool save_refs) {
    if ( save_modes ) {
        fcs_mode = fcs_node.getString("mode");
        nav_mode = nav_node.getString("mode");
    }
    if ( save_circle ) {
        circle_lon_deg = circle_node.getDouble("longitude_deg");
        circle_lat_deg = circle_node.getDouble("latitude_deg");
        circle_direction = circle_node.getInt("direction");
        circle_radius_m = circle_node.getDouble("radius_m");
    }
    if ( save_refs ) {
        ref_agl_m = refs_node.getDouble("altitude_agl_m");
        ref_speed_mps = refs_node.getDouble("airspeed_mps");
    }
}

void MissionState::restore() {

}

vector<MissionState> state_stack;

void save_mission_state(bool modes=false, bool circle=false, bool refs=false) {
    MissionState current_state = MissionState(modes, circle, refs);
    state_stack.push_back(current_state);
}

void restore_mission_state() {
    MissionState saved_state = state_stack.back();
    state_stack.pop_back();
    saved_state.restore();
}