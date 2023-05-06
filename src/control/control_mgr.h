#pragma once

#include "../props2.h"

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

    PropertyNode status_node;
    PropertyNode ap_node;
    PropertyNode targets_node;
    PropertyNode tecs_node;
    PropertyNode task_node;
    PropertyNode pilot_node;
    PropertyNode flight_node;
    PropertyNode engine_node;
    PropertyNode route_node;
    PropertyNode home_node;
    PropertyNode circle_node;
    PropertyNode pos_node;
    PropertyNode switches_node;

    void copy_pilot_inputs();
};
