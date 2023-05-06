#include "../props2.h"

class airdata_helper_t {

private:

    const double kt2mps = 0.5144444444444444444;

    PropertyNode airdata_node;

    // is airborne
    float up_mps = 6.0;
    float down_mps = 4.0;
    float up_m = 6;
    float down_m = 3;
    bool is_airborne = false;

    // flight timer
    uint32_t last_millis = 0;
    uint32_t flight_millis = 0;

public:

    void init();
    void update();

};
