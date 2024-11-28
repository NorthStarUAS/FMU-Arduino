#include "../nst_messages.h"

class packer_t {

public:

    nst_message::airdata_v9_t air_msg;
    nst_message::environment_v1_t environment_msg;
    nst_message::effectors_v1_t eff_msg;
    nst_message::event_v3_t event_msg;
    uint32_t gps_last_millis = 0;
    nst_message::gps_v5_t gps_msg;
    nst_message::imu_v6_t imu_msg;
    nst_message::inceptors_v2_t inceptor_msg;
    nst_message::fcs_outputs_v1_t outputs_msg;
    uint16_t route_counter = 0;
    nst_message::mission_v1_t mission_msg;
    nst_message::nav_v6_t nav_msg;
    nst_message::nav_metrics_v6_t nav_metrics_msg;
    nst_message::power_v2_t power_msg;
    nst_message::fcs_refs_v1_t refs_msg;
    uint32_t bytes_last_millis = 0;
    nst_message::status_v8_t status_msg;

    void update();

private:

    void pack_airdata();
    void pack_environment();
    void pack_effectors();
    void pack_event();
    void pack_gps();
    void pack_imu();
    void pack_inceptors();
    void pack_outputs();
    void pack_mission();
    void pack_nav();
    void pack_nav_metrics();
    void pack_power();
    void pack_refs();
    void pack_status();

};