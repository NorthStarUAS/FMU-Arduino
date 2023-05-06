#include <Arduino.h>

#include "airdata_helper.h"
#include "ground.h"
#include "switches.h"
#include "wind.h"

class state_mgr_t {

private:

    airdata_helper_t airdata;
    ground_est_t ground;
    switches_t switches;
    wind_est_t wind;

public:

    void init();
    void update(float dt);

};

extern state_mgr_t state_mgr;