
#include "../nodes.h"
#include "state.h"

MissionState::MissionState() {
    state_stack.clear();
}

void MissionState::save(bool save_modes, bool save_circle, bool save_refs) {
    State state;
    if ( save_modes ) {
        state.fcs_mode = fcs_node.getString("mode");
        state.mission_mode = mission_node.getString("mode");
    }
    if ( save_circle ) {
        state.circle_lon_deg = circle_node.getDouble("longitude_deg");
        state.circle_lat_deg = circle_node.getDouble("latitude_deg");
        state.circle_direction = circle_node.getInt("direction");
        state.circle_radius_m = circle_node.getDouble("radius_m");
    }
    if ( save_refs ) {
        state.ref_agl_m = refs_node.getDouble("altitude_agl_m");
        state.ref_speed_mps = refs_node.getDouble("airspeed_mps");
    }
    state_stack.push_back(state);
}

void MissionState::restore() {
    State state = state_stack.back();
    state_stack.pop_back();

    if ( state.fcs_mode.length() ) {
        fcs_node.setString("mode", state.fcs_mode);
    }
    if ( state.mission_mode.length() ) {
        mission_node.setString("mode", state.mission_mode);
    }
    if ( fabs(state.circle_lon_deg) > 0.1 or fabs(state.circle_lat_deg) > 0.1 ) {
        circle_node.setDouble("longitude_deg", state.circle_lon_deg);
        circle_node.setDouble("latitude_deg", state.circle_lat_deg);
        circle_node.setInt("direction", state.circle_direction);
        circle_node.setDouble("radius_m", state.circle_radius_m);
    }
    if ( state.ref_agl_m > 0.1 or state.ref_speed_mps > 0.1 ) {
        refs_node.setDouble("altitude_agl_m", state.ref_agl_m);
        refs_node.setDouble("airspeed_mps", state.ref_speed_mps);
    }
}