#include "../ns_messages.h"

class packer_t {

public:

    ns_message::inceptors_v2_t inceptor_msg;
    ns_message::effectors_v1_t eff_msg;
    ns_message::imu_v6_t imu_msg;
    ns_message::gps_v5_t gps_msg;
    uint32_t gps_last_millis = 0;
    ns_message::nav_v6_t nav_msg;
    ns_message::nav_metrics_v6_t metrics_msg;
    ns_message::airdata_v8_t air_msg;
    ns_message::fcs_refs_v1_t refs_msg;
    ns_message::mission_v1_t mission_msg;
    ns_message::power_v1_t power_msg;
    ns_message::status_v7_t status_msg;
    ns_message::event_v3_t event_msg;

    void update();

private:

    void pack_inceptors();
    void pack_effectors();
    void pack_imu();
    void pack_gps();
    void pack_nav();
    void pack_nav_metrics();
    void pack_airdata();
    void pack_refs();
    void pack_mission();
    void pack_power();
    void pack_status();
    void pack_event();

};