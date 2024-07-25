/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include <Arduino.h>

#include "../nodes.h"

#include "../guidance/route_mgr.h"
#include "../nav/nav_mgr.h"                // reset ekf
#include "../nav/nav_constants.h"
#include "../sensors/imu_mgr.h"            // reset gyros
#include "../sensors/pilot.h"              // update_ap()

#include "relay.h"
#include "ns_messages.h"
#include "message_link.h"

message_link_t::message_link_t() {}
message_link_t::~message_link_t() {}

void message_link_t::init(uint8_t port, uint32_t baud, string relay_name) {
    // port: 0 = usb/console, 1 = telem 1 (host), 2 = telem 2 (gcs)
    // telemetry baud = 57600 (or 115200), host baud = 500,000
    saved_port = port;
    if ( serial.open(baud, port) ) {
        printf("opened message_link port: %d @ %ld baud\n", port, baud);
        delay(100);
    } else {
        printf("ERROR opening message_link port: %d @ %ld baud\n", port, baud);
        delay(1000);
    }
    relay_id = relay_name;
    if ( relay_id == "host" ) {
        relay.set_host_link(&serial);
    } else if ( relay_id == "gcs" ) {
        relay.set_gcs_link(&serial);
    }

    // fixme: make externally configurable?
    if ( baud <= 115200 ) {
        // setup rates for a slower telemetry connection
        airdata_limiter = RateLimiter(2);
        ap_limiter = RateLimiter(2);
        eff_limiter = RateLimiter(4);
        gps_limiter = RateLimiter(2.5);
        imu_limiter = RateLimiter(4);
        mission_limiter = RateLimiter(2);
        nav_limiter = RateLimiter(10);
        nav_metrics_limiter = RateLimiter(0.5);
        pilot_limiter = RateLimiter(4);
        power_limiter = RateLimiter(1);
        status_limiter = RateLimiter(0.1);
    } else {
        // setup rates for a full speed host connection
        airdata_limiter = RateLimiter(0);
        ap_limiter = RateLimiter(-1);  // don't send
        eff_limiter = RateLimiter(-1); // don't send (maybe send for logging?)
        gps_limiter = RateLimiter(0);
        imu_limiter = RateLimiter(0);
        mission_limiter = RateLimiter(0);
        nav_limiter = RateLimiter(0);
        nav_metrics_limiter = RateLimiter(0.5);
        pilot_limiter = RateLimiter(0);
        power_limiter = RateLimiter(0);
        status_limiter = RateLimiter(0.5);
    }
}

void message_link_t::update() {
    if ( airdata_limiter.update() ) {
        output_counter += write_airdata();
    }
    if ( ap_limiter.update() ) {
        output_counter += write_ap();
    }
    if ( eff_limiter.update() ) {
        output_counter += write_effectors();
    }
    if ( gps_limiter.update() ) {
        output_counter += write_gps();
    }
    if ( nav_limiter.update() ) {
        output_counter += write_nav();
    }
    if ( nav_metrics_limiter.update() ) {
        output_counter += write_nav_metrics();
    }
    if ( pilot_limiter.update() ) {
        output_counter += write_pilot();
    }
    if ( power_limiter.update() ) {
        output_counter += write_power();
    }
    if ( status_limiter.update() ) {
        int len = write_status();
        output_counter = len;   // start over counting bytes
    }
    // tradition has us writing the imu packet last so a full rate
    // receiver could use it as an end of frame marker if it wanted
    // to.
    if ( imu_limiter.update() ) {
        output_counter += write_imu();
    }
}

bool message_link_t::parse_message( uint8_t id, uint8_t *buf, uint8_t message_size ) {
    bool result = false;
    //printf("message id: %d  len: %d\n", id, message_size);
    if ( id == ns_message::inceptors_v1_id ) {
        static ns_message::inceptors_v1_t inceptors;
        inceptors.unpack(buf, message_size);
        if ( message_size == inceptors.len ) {
            pilot.update_ap(&inceptors);
            result = true;
        }
    } else if ( id == ns_message::command_v1_id ) {
        ns_message::command_v1_t msg;
        msg.unpack(buf, message_size);
        printf("received command: %s %d\n",
                        msg.message.c_str(), msg.sequence_num);
        uint8_t command_result = 0;
        if ( last_command_seq_num != msg.sequence_num ) {
            last_command_seq_num = msg.sequence_num;
            if ( msg.message == "hb" ) {
                command_result = 1;
            } else if ( msg.message == "zero_gyros" ) {
                imu_mgr.gyros_calibrated = 0;   // start state
                command_result = 1;
            } else if ( msg.message == "reset_ekf" ) {
                nav_mgr.reinit();
                command_result = 1;
            } else if ( msg.message.substr(0, 4) == "get " ) {
                string path = msg.message.substr(4);
                // printf("cmd: get  node: %s\n", path.c_str());
                PropertyNode node(path);
                ns_message::command_v1_t reply;
                reply.sequence_num = 0;
                reply.message = "set " + path + " " + node.get_json_string();
                reply.pack();
                serial.write_packet( reply.id, reply.payload, reply.len);
                command_result = 1;
            } else {
                // printf("unknown message: %s, relaying to host\n", msg.message.c_str());
                if ( relay_id == "gcs" ) {
                    relay.forward_packet(relay_t::dest_enum::host_dest,
                                         id, buf, message_size);
                }
                command_result = 1;
            }
        } else {
            printf("ignoring duplicate command\n");
            command_result = 1;
        }
        write_ack( msg.sequence_num, command_result );
        result = true;
    } else if ( id == ns_message::ap_targets_v1_id ) {
        ns_message::ap_targets_v1_t ap_msg;
        ap_msg.unpack(buf, message_size);
        ap_msg.msg2props(targets_node);
    } else if ( id == ns_message::mission_v1_id ) {
        // relay directly to gcs
        if ( relay_id == "host" ) {
            relay.forward_packet(relay_t::dest_enum::gcs_dest,
                                 id, buf, message_size);
        }
        // this is the messy message
        ns_message::mission_v1_t mission;
        mission.unpack(buf, message_size);
        if ( message_size == mission.len ) {
            task_node.setDouble("flight_timer", mission.flight_timer);
            task_node.setString("current_task", mission.task_name);
            task_node.setInt("task_attribute", mission.task_attribute);
            route_node.setInt("target_waypoint_idx", mission.target_waypoint_idx);
            double wp_lon = mission.wp_longitude_raw / 10000000.0l;
            double wp_lat = mission.wp_latitude_raw / 10000000.0l;
            int wp_index = mission.wp_index;
            PropertyNode wp_node;
            if ( mission.route_size != route_mgr.get_active_size() ) {
                // route size change, zero all the waypoint coordinates
                route_mgr.set_active_size( mission.route_size );
                for ( int i = 0; i < mission.route_size; i++ ) {
                    route_mgr.set_wp( i, waypoint_t() );
                }
            }
            if ( wp_index < mission.route_size ) {
                route_mgr.set_wp( wp_index, waypoint_t(1, wp_lon, wp_lat) );
            } else if ( wp_index == 65534 ) {
                circle_node.setDouble("longitude_deg", wp_lon);
                circle_node.setDouble("latitude_deg", wp_lat);
                circle_node.setDouble("radius_m", mission.task_attribute / 10.0);
            } else if ( wp_index == 65535 ) {
                home_node.setDouble("longitude_deg", wp_lon);
                home_node.setDouble("latitude_deg", wp_lat);
            }
            result = true;
        }
    } else if ( id == ns_message::airdata_v8_id ) {
        if ( status_node.getBool("HIL_mode") ) {
            ns_message::airdata_v8_t msg;
            msg.unpack(buf, message_size);
            // don't autofill the property tree, just pick the values the flight
            // computer needs so we can compute the rest ourselves as if we are
            // running with real sensors
            // msg.msg2props(airdata_node);
            uint32_t airdata_millis = millis();  // force our own timestamp
            airdata_node.setUInt("millis", airdata_millis);
            airdata_node.setDouble("timestamp", airdata_millis / 1000.0);
            airdata_node.setDouble("baro_press_pa", msg.baro_press_pa);
            airdata_node.setDouble("diff_press_pa", msg.diff_press_pa);
            airdata_node.setDouble("air_temp_C", msg.air_temp_C);
            airdata_node.setDouble("airspeed_mps", msg.airspeed_mps);
            airdata_node.setDouble("altitude_agl_m", msg.altitude_agl_m);
            airdata_node.setDouble("altitude_true_m", msg.altitude_true_m);
            airdata_node.setDouble("altitude_ground_m", msg.altitude_ground_m);
            // node.setUInt("is_airborne", self.is_airborne)
            // node.setUInt("flight_timer_millis", self.flight_timer_millis)
            // node.setDouble("wind_dir_deg", self.wind_dir_deg)
            // node.setDouble("wind_speed_mps", self.wind_speed_mps)
            // node.setDouble("pitot_scale_factor", self.pitot_scale_factor)
            airdata_node.setUInt("error_count", msg.error_count);
        }
    } else if ( id == ns_message::gps_v5_id ) {
        if ( status_node.getBool("HIL_mode") ) {
            ns_message::gps_v5_t msg;
            msg.unpack(buf, message_size);
            msg.msg2props(gps_node);
            uint32_t gps_millis = millis();  // force our own timestamp
            gps_node.setUInt("millis", gps_millis);
            gps_node.setBool("settle", true);
            gps_node.setDouble("timestamp", gps_millis / 1000.0);
            gps_node.setDouble("latitude_deg", (double)(gps_node.getInt("latitude_raw")) / 10000000.0l);
            gps_node.setDouble("longitude_deg", (double)(gps_node.getInt("longitude_raw")) / 10000000.0l);
            // gps_node.pretty_print();
        }
    } else if ( id == ns_message::imu_v6_id ) {
        // special logic to switch system to HIL mode if we are receiving IMU
        // data as external messages, but only if we aren't already flying
        //
        // FIXME: let's add extra protections here (somewhere?) against
        // inadvertently switching to HIL or accepting external data while in
        // flight.
        if ( !airdata_node.getBool("is_airborne") ) {
            status_node.setBool("HIL_mode", true);
        }

        if ( status_node.getBool("HIL_mode") ) {
            ns_message::imu_v6_t msg;
            msg.unpack(buf, message_size);
            msg.msg2props(imu_node);
            uint32_t imu_millis = millis();  // force our own timestamp
            imu_node.setUInt("millis", imu_millis); // force our own timestamp
            imu_node.setDouble("timestamp", imu_millis / 1000.0);
            imu_node.setUInt("gyros_calibrated", 2);  // flag gyros from external source as calibrated
            // imu_node.pretty_print();
        }
    } else if ( id == ns_message::inceptors_v1_id ) {
        if ( status_node.getBool("HIL_mode") ) {
            ns_message::inceptors_v1_t msg;
            msg.unpack(buf, message_size);
            msg.msg2props(pilot_node);
            uint32_t pilot_millis = millis();  // force our own timestamp
            pilot_node.setUInt("millis", pilot_millis); // force our own timestamp
            pilot_node.setDouble("timestamp", pilot_millis / 1000.0);
        }
    } else if ( id == ns_message::power_v1_id ) {
        if ( status_node.getBool("HIL_mode") ) {
            ns_message::power_v1_t msg;
            msg.unpack(buf, message_size);
            msg.msg2props(power_node);
            uint32_t power_millis = millis();  // force our own timestamp
            power_node.setUInt("millis", power_millis); // force our own timestamp
            power_node.setDouble("timestamp", power_millis / 1000.0);
        }
    } else {
        printf("unknown message id: %d len: %d\n", id, message_size);
    }
    return result;
}

// return an ack of a message received
int message_link_t::write_ack( uint16_t sequence_num, uint8_t result )
{
    static ns_message::ack_v1_t ack;
    ack.sequence_num = sequence_num;
    ack.result = result;
    ack.pack();
    return serial.write_packet( ack.id, ack.payload, ack.len);
}

// final effector commands
int message_link_t::write_effectors()
{
    static ns_message::effectors_v1_t eff_msg;
    eff_msg.props2msg(effectors_node);
    eff_msg.pack();
    return serial.write_packet( eff_msg.id, eff_msg.payload, eff_msg.len);
}

// pilot manual (rc receiver) data
int message_link_t::write_pilot()
{
    static ns_message::pilot_v4_t pilot_msg;
    pilot_msg.props2msg(pilot_node);
    pilot_msg.master_switch = switches_node.getBool("master_switch");
    pilot_msg.throttle_safety = switches_node.getBool("throttle_safety");
    pilot_msg.pack();
    return serial.write_packet( pilot_msg.id, pilot_msg.payload, pilot_msg.len);
}

int message_link_t::write_imu()
{
    static ns_message::imu_v6_t imu_msg;
    imu_msg.props2msg(imu_node);
    imu_msg.pack();
    return serial.write_packet( imu_msg.id, imu_msg.payload, imu_msg.len );
}

int message_link_t::write_gps()
{
    static ns_message::gps_v5_t gps_msg;
    if ( gps_node.getUInt("millis") != gps_last_millis ) {
        gps_last_millis = gps_node.getUInt("millis");
        gps_msg.props2msg(gps_node);
        gps_msg.pack();
        return serial.write_packet( gps_msg.id, gps_msg.payload, gps_msg.len );
    } else {
        return 0;
    }
}

// nav (ekf) data
int message_link_t::write_nav()
{
    static ns_message::nav_v6_t nav_msg;
    nav_msg.props2msg(nav_node);
    nav_msg.pack();
    return serial.write_packet( nav_msg.id, nav_msg.payload, nav_msg.len );
}

// nav (ekf) metrics
int message_link_t::write_nav_metrics()
{
    static ns_message::nav_metrics_v6_t metrics_msg;
    metrics_msg.props2msg(nav_node);
    metrics_msg.metrics_millis = nav_node.getUInt("millis");
    metrics_msg.pack();
    return serial.write_packet( metrics_msg.id, metrics_msg.payload, metrics_msg.len );
}

int message_link_t::write_airdata()
{
    static ns_message::airdata_v8_t air_msg;
    air_msg.props2msg(airdata_node);
    air_msg.altitude_ground_m = pos_node.getDouble("altitude_ground_m");
    air_msg.pack();
    return serial.write_packet( air_msg.id, air_msg.payload, air_msg.len );
}

// autopilot targets / status
int message_link_t::write_ap()
{
    ns_message::ap_targets_v1_t ap_msg;
    ap_msg.props2msg(targets_node);
    ap_msg.millis = imu_node.getUInt("millis");
    ap_msg.pack();
    return serial.write_packet( ap_msg.id, ap_msg.payload, ap_msg.len );
}

int message_link_t::write_power()
{
    static ns_message::power_v1_t power_msg;
    power_msg.props2msg(power_node);
    power_msg.pack();
    return serial.write_packet( power_msg.id, power_msg.payload, power_msg.len );
}

// system status
int message_link_t::write_status()
{
    static ns_message::status_v7_t status_msg;

    // estimate output byte rate
    uint32_t current_time = millis(); // fixme use elapsedmillis(), not a variable called that...
    uint32_t elapsed_millis = current_time - bytes_last_millis;
    bytes_last_millis = current_time;
    uint32_t byte_rate = output_counter * 1000 / elapsed_millis;
    status_node.setUInt("byte_rate", byte_rate);

    status_msg.props2msg(status_node);
    status_msg.millis = current_time;
    status_msg.pack();
    return serial.write_packet( status_msg.id, status_msg.payload, status_msg.len );
}

void message_link_t::read_commands() {
    while ( serial.update() ) {
        parse_message( serial.pkt_id, serial.payload, serial.pkt_len );
    }
}
