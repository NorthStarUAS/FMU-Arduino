#include "../nodes.h"
#include "../mission/mission_mgr.h"
#include "events.h"
#include "packer.h"

void packer_t::update() {
    pack_airdata();
    pack_effectors();
    pack_event();
    pack_gps();
    pack_imu();
    pack_inceptors();
    pack_mission();
    pack_nav();
    pack_nav_metrics();
    pack_power();
    pack_refs();
    pack_status();
}

void packer_t::pack_airdata() {
    air_msg.props2msg(airdata_node);
    air_msg.pack();
}

// final effector commands
void packer_t::pack_effectors() {
    eff_msg.props2msg(effectors_node);
    eff_msg.pack();
}

// pack one event per iteration and assume the message_link instances and logger
// catch it on the same iteration.
void packer_t::pack_event() {
    if ( event_mgr != nullptr and event_mgr->event_list.size()) {
        event_msg.millis = millis();
        event_msg.message = event_mgr->event_list[0];
        event_msg.pack();
        event_mgr->pop_front();
    } else {
        event_msg.millis = 0;
    }
}

void packer_t::pack_gps() {
    if ( gps_node.getUInt("millis") != gps_last_millis ) {
        gps_last_millis = gps_node.getUInt("millis");
        gps_msg.props2msg(gps_node);
        gps_msg.pack();
    }
}

void packer_t::pack_imu() {
    imu_msg.props2msg(imu_node);
    imu_msg.pack();
}

// inceptors
void packer_t::pack_inceptors() {
    inceptor_msg.props2msg(inceptors_node);
    inceptor_msg.pack();
}

// mission values
void packer_t::pack_mission() {
    // this is the messy message we need to assemble the old fashioned way
    mission_msg.millis = imu_node.getUInt("millis");
    mission_msg.task_name = mission_node.getString("task");
    mission_msg.task_attribute = 0; // fixme task attribute?
    unsigned int route_size = route_node.getUInt("route_size");
    mission_msg.route_size = route_size;
    mission_msg.target_wpt_idx = route_node.getUInt("target_wpt_idx");;

    mission_msg.wpt_index = route_counter;
    if ( route_counter < route_size ) {
        // it is unfortunate that get_wp converts from raw to deg and we have to
        // convert back to raw for the message.
        coord_t coord = mission_mgr->route_mgr.get_wp(route_counter);
        mission_msg.wpt_longitude_raw = coord.lon_deg * 10000000;
        mission_msg.wpt_latitude_raw = coord.lat_deg * 10000000;
        mission_msg.task_attribute = 0;
        route_counter++;
    } else if ( route_counter >= route_size and route_counter < 65534 ) {
        // eat a null message if we get caught in this condition
        mission_msg.wpt_longitude_raw = 0;
        mission_msg.wpt_latitude_raw = 0;
        mission_msg.task_attribute = 0;
        route_counter = 65534;
    } else if ( route_counter == 65534 ) {
        mission_msg.wpt_longitude_raw = circle_node.getDouble("longitude_deg") * 10000000;
        mission_msg.wpt_latitude_raw = circle_node.getDouble("latitude_deg") * 10000000;
        mission_msg.task_attribute = circle_node.getDouble("radius_m");
        if ( circle_node.getString("direction") == "right" ) {
            mission_msg.task_attribute += 30000;
        }
        route_counter = 65535;
    } else if ( route_counter == 65535 ) {
        if ( home_node.getBool("valid") ) {
            mission_msg.wpt_longitude_raw = home_node.getDouble("longitude_deg") * 10000000;
            mission_msg.wpt_latitude_raw = home_node.getDouble("latitude_deg") * 10000000;
            mission_msg.task_attribute = home_node.getDouble("azimuth_deg");
        } else {
            mission_msg.wpt_longitude_raw = 0;
            mission_msg.wpt_latitude_raw = 0;
            mission_msg.task_attribute = 0;
        }
        route_counter = 0;
    }
    mission_msg.pack();
}

// nav (ekf) data
void packer_t::pack_nav() {
    nav_msg.props2msg(nav_node);
    nav_msg.sequence_num = comms_node.getUInt("last_command_seq_num");
    nav_msg.pack();
}

// nav (ekf) metrics
void packer_t::pack_nav_metrics() {
    nav_metrics_msg.props2msg(nav_node);
    nav_metrics_msg.metrics_millis = nav_node.getUInt("millis");
    nav_metrics_msg.pack();
}

void packer_t::pack_power() {
    power_msg.props2msg(power_node);
    power_msg.pack();
}

// fcs reference values
void packer_t::pack_refs() {
    refs_msg.props2msg(refs_node);
    refs_msg.millis = imu_node.getUInt("millis");
    refs_msg.pack();
}

// system status
void packer_t::pack_status() {
    uint32_t current_time = millis(); // fixme use elapsedmillis(), not a variable called that...
    status_msg.props2msg(status_node);
    status_msg.millis = current_time;
    // estimate output byte rate
    uint32_t elapsed_millis = current_time - bytes_last_millis;
    bytes_last_millis = current_time;
    uint32_t byte_rate = comms_node.getUInt("bytes_to_gcs") * 1000 / elapsed_millis;
    comms_node.setUInt("bytes_to_gcs", 0);
    status_msg.link_state = comms_node.getBool("link_state");
    status_msg.byte_rate = byte_rate;
    status_msg.pack();
}

