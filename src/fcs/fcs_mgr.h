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
    AuraAutopilot ap;

    void copy_pilot_inputs();
};
