#include "nodes.h"

// Configuration
PropertyNode config_node;
PropertyNode config_nav_node;
PropertyNode config_power_node;
PropertyNode config_eff_gains_node;
PropertyNode config_L1_node;
PropertyNode config_tecs_node;
PropertyNode imu_calib_node;

// Sensors
PropertyNode airdata_node;
PropertyNode gps_node;
PropertyNode imu_node;
PropertyNode pilot_node;
PropertyNode power_node;
PropertyNode rcin_node;
PropertyNode switches_node;

// INS/GNSS
PropertyNode nav_node;

PropertyNode orient_node;
PropertyNode pos_node;
PropertyNode vel_node;
PropertyNode wind_node;

// Inceptors and Effectors
PropertyNode effectors_node;
PropertyNode engine_node;
PropertyNode flight_node;

// Status and Comms
PropertyNode comms_node;
PropertyNode status_node;

// Control Laws
PropertyNode ap_node;
PropertyNode targets_node;
PropertyNode tecs_node;

// Mission and Tasks
PropertyNode guidance_node;
PropertyNode circle_node;
PropertyNode home_node;
PropertyNode route_node;
PropertyNode task_node;

// FUTURE
PropertyNode sim_node;

void PropertyNodes_init() {
    // Configuration
    config_node = PropertyNode("/config");
    config_nav_node = PropertyNode("/config/nav");
    config_power_node = PropertyNode("/config/power");
    config_eff_gains_node = PropertyNode("/config/pwm");
    config_L1_node = PropertyNode("/config/autopilot/L1_controller");
    config_tecs_node = PropertyNode("/config/autopilot/TECS");
    imu_calib_node = PropertyNode("/config/imu/calibration");

    // Sensors
    airdata_node = PropertyNode("/sensors/airdata");
    gps_node = PropertyNode("/sensors/gps");
    imu_node = PropertyNode("/sensors/imu");
    pilot_node = PropertyNode("/pilot");
    power_node = PropertyNode("/sensors/power");
    rcin_node = PropertyNode("/sensors/rc-input");
    switches_node = PropertyNode("/switches");

    // INS/GNSS
    nav_node = PropertyNode("/filters/nav");

    orient_node = PropertyNode("/orientation");
    pos_node = PropertyNode("/position");
    vel_node = PropertyNode("/velocity");
    wind_node = PropertyNode("/filters/wind");

    // Inceptors and Effectors
    effectors_node = PropertyNode("/effectors");
    engine_node = PropertyNode( "/controls/engine" );
    flight_node = PropertyNode( "/controls/flight" );

    // Status and Comms
    comms_node = PropertyNode("/comms");
    status_node = PropertyNode("/status");

    // Control Laws
    ap_node = PropertyNode("/autopilot");
    targets_node = PropertyNode("/autopilot/targets");
    tecs_node = PropertyNode("/autopilot/tecs");

    // Mission and Tasks
    guidance_node = PropertyNode("/guidance");
    circle_node = PropertyNode("/task/circle/active");
    home_node = PropertyNode("/task/home");
    route_node = PropertyNode("/task/route");
    task_node = PropertyNode("/task");

    // FUTURE
    sim_node = PropertyNode("/sim");
}
