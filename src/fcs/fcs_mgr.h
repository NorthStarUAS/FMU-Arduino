#pragma once

#include "ap.h"

class fcs_mgr_t {

public:

    fcs_mgr_t() {};
    ~fcs_mgr_t() {};
    void init();
    void reset();
    void update( float dt );

private:

    AutoPilot ap;
    bool last_master_switch = false;

    void copy_pilot_inputs();
};

extern fcs_mgr_t *fcs_mgr;