#pragma once

#include "ap.h"

class control_mgr_t {
public:
    control_mgr_t() {};
    ~control_mgr_t() {};
    void init();
    void reset();
    void update( float dt );

private:
    AuraAutopilot ap;

    void copy_pilot_inputs();
};
