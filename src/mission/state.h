#include <string>
#include <vector>
using std::string;
using std::vector;

class MissionState {

public:

    MissionState();

    void save(bool modes=false, bool circle=false, bool refs=false);
    void restore();

private:

    struct State {
        string fcs_mode = "";
        string mission_mode = "";

        double circle_lon_deg = 0.0;
        double circle_lat_deg = 0.0;
        int circle_direction = 1;
        float circle_radius_m = 100;

        float ref_agl_m = 0.0;
        float ref_speed_mps = 0.0;
    };

    vector<State> state_stack;

};

