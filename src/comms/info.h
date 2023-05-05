#pragma once

#include "../props2.h"

class info_t {
public:
    void init();
    void write_pilot_in_ascii();
    void write_actuator_out_ascii();
    void write_imu_ascii();
    void write_gps_ascii();
    void write_nav_ascii();
    void write_nav_stats_ascii();
    void write_airdata_ascii();
    void write_power_ascii();
    void write_status_info_ascii();

private:
    PropertyNode config_node;
    PropertyNode effector_node;
    PropertyNode nav_node;
    PropertyNode airdata_node;
    PropertyNode gps_node;
    PropertyNode imu_node;
    PropertyNode pilot_node;
    PropertyNode power_node;
    PropertyNode switches_node;
};
