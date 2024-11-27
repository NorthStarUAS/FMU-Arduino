#pragma once

#include <Arduino.h>

#include "airdata_helper.h"
#include "ground.h"
#include "wind.h"

class env_mgr_t {

private:

    airdata_helper_t airdata;
    ground_est_t ground;
    wind_est_t wind;

public:

    void init();
    void update(float dt);

};

extern env_mgr_t env_mgr;