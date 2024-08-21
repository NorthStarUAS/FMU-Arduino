#include "nodes.h"

// Configuration
PropertyNode config_node;
PropertyNode config_L1_node;
PropertyNode config_tecs_node;
PropertyNode imu_calib_node;

// Sensors
PropertyNode airdata_node;
PropertyNode gps_node;
PropertyNode imu_node;
PropertyNode inceptors_node;
PropertyNode power_node;
PropertyNode rcin_node;

// INS/GNSS
PropertyNode nav_node;

// State
PropertyNode wind_node;

// Status and Comms
PropertyNode comms_node;
PropertyNode status_node;

// Flight Control System
PropertyNode fcs_node;
PropertyNode locks_node;
PropertyNode refs_node;
PropertyNode tecs_node;
PropertyNode outputs_node;
PropertyNode effectors_node;

// Mission and Tasks
PropertyNode mission_node;
PropertyNode circle_node;
PropertyNode home_node;
PropertyNode route_node;
PropertyNode startup_node;
PropertyNode task_node;

// Performance / debug
PropertyNode profile_node;

void PropertyNodes_init() {
    // Configuration
    config_node = PropertyNode("/config");
    config_L1_node = PropertyNode("/config/autopilot/L1_controller");
    config_tecs_node = PropertyNode("/config/autopilot/TECS");
    imu_calib_node = PropertyNode("/config/imu/calibration");

    // Sensors
    airdata_node = PropertyNode("/sensors/airdata");
    gps_node = PropertyNode("/sensors/gps");
    imu_node = PropertyNode("/sensors/imu");
    inceptors_node = PropertyNode("/sensors/inceptors");
    power_node = PropertyNode("/sensors/power");
    rcin_node = PropertyNode("/sensors/rc_input");

    // INS/GNSS
    nav_node = PropertyNode("/filters/nav");

    // Status and Comms
    comms_node = PropertyNode("/comms");
    status_node = PropertyNode("/status");

    // Flight Control Laws
    fcs_node = PropertyNode("/fcs");
    locks_node = PropertyNode("/fcs/locks");
    refs_node = PropertyNode("/fcs/refs");
    tecs_node = PropertyNode("/fcs/tecs");
    outputs_node = PropertyNode("/fcs/outputs");
    effectors_node = PropertyNode("/fcs/effectors");

    // Mission and Tasks
    mission_node = PropertyNode("/mission");
    circle_node = PropertyNode("/mission/circle");
    home_node = PropertyNode("/mission/home");
    route_node = PropertyNode("/mission/route");
    startup_node = PropertyNode("/mission/startup");
    task_node = PropertyNode("/task");

    // Performance / debug
    profile_node = PropertyNode("/profile");
}
