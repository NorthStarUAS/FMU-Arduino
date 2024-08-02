#include <string>
using std::string;

class MissionState {

    MissionState(bool save_modes, bool save_circle, bool save_refs);
    void restore();

private:

    string fcs_mode = "";
    string nav_mode = "";

    double circle_lon_deg = 0.0;
    double circle_lat_deg = 0.0;
    int circle_direction = 1;
    float circle_radius_m = 100;

    float ref_agl_m = 0.0;
    float ref_speed_mps = 0.0;

};

void save_mission_state(bool modes=false, bool circle=false, bool refs=false);
void restore_mission_state();