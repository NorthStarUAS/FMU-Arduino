/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include <Arduino.h>

#include "../nodes.h"
#include "../nst_messages.h"

#include "../mission/mission_mgr.h"
#include "../nav/nav_mgr.h"                // reset ekf
#include "../nav/nav_constants.h"
#include "../sensors/sensor_mgr.h"         // reset gyros

#include "comms_mgr.h"
#include "events.h"
#include "remote_command.h"
#include "message_link.h"

void message_link_t::init(uint8_t port, uint32_t baud ) {
    // port: 0 = usb/console, 1 = telem 1 (host), 2 = telem 2 (gcs)
    // telemetry baud = 57600 (or 115200), host baud = 500,000
    saved_port = port;
    saved_baud = baud;
    if ( serial.open(baud, port) ) {
        printf("opened message_link port: %d @ %ld baud\n", port, baud);
    } else {
        printf("ERROR opening message_link port: %d @ %ld baud\n", port, baud);
    }

    limiter_50hz = RateLimiter(50);
    limiter_10hz = RateLimiter(10);
    limiter_4hz = RateLimiter(4);
    limiter_2_5hz = RateLimiter(2.5);
    limiter_2hz = RateLimiter(2);
    limiter_1sec = RateLimiter(1);
    limiter_2sec = RateLimiter(0.5);
    limiter_10sec = RateLimiter(0.1);
}

void message_link_t::write_messages() {
    // fixme: make externally configurable?
    if ( saved_baud <= 115200 ) {
        write_events();
        if ( limiter_10hz.update() ) {
            write_nav();
        }
        if ( limiter_4hz.update() ) {
            write_effectors();
            write_imu();
            write_inceptors();
            write_outputs();
        }
        if ( limiter_2_5hz.update() ) {
            write_gps();
        }
        if ( limiter_2hz.update() ) {
            write_airdata();
            write_environment();
            write_refs();
            write_mission();
        }
        if ( limiter_1sec.update() ) {
            write_power();
        }
        if ( limiter_2sec.update() ) {
            write_nav_metrics();
        }
        if ( limiter_10sec.update() ) {
            write_status();
        }
    } else {
        write_events();
        if ( limiter_50hz.update() ) {
            write_airdata();
            write_environment();
            write_effectors();
            write_gps();
            write_imu();
            write_inceptors();
            write_outputs();
            write_mission();
            write_nav();
            write_power();
            write_refs();
        }
        if ( limiter_2sec.update() ) {
            write_nav_metrics();
            write_status();
        }
    }

    write_bytes();
}

// return an ack of a message received
void message_link_t::write_ack( uint16_t sequence_num, uint8_t result ) {
    nst_message::ack_v1_t ack;
    ack.sequence_num = sequence_num;
    ack.result = result;
    ack.pack();
    send_packet( ack.id, ack.payload, ack.len);
}

void message_link_t::write_airdata() {
    nst_message::airdata_v9_t &air_msg = comms_mgr->packer.air_msg;
    send_packet( air_msg.id, air_msg.payload, air_msg.len );
}

void message_link_t::write_environment() {
    nst_message::environment_v1_t &env_msg = comms_mgr->packer.environment_msg;
    send_packet( env_msg.id, env_msg.payload, env_msg.len );
}

// final effector commands
void message_link_t::write_effectors() {
    nst_message::effectors_v1_t &eff_msg = comms_mgr->packer.eff_msg;
    send_packet( eff_msg.id, eff_msg.payload, eff_msg.len);
}

void message_link_t::write_events() {
    nst_message::event_v3_t &event_msg = comms_mgr->packer.event_msg;
    if ( event_msg.millis > event_last_millis ) {
        event_last_millis = event_msg.millis;
        send_packet( event_msg.id, event_msg.payload, event_msg.len );
    }
}

void message_link_t::write_gps() {
    nst_message::gps_v5_t &gps_msg = comms_mgr->packer.gps_msg;
    if ( gps_msg.millis > gps_last_millis ) {
        gps_last_millis = gps_msg.millis;
        send_packet( gps_msg.id, gps_msg.payload, gps_msg.len );
    }
}

void message_link_t::write_imu() {
    nst_message::imu_v6_t &imu_msg = comms_mgr->packer.imu_msg;
    send_packet( imu_msg.id, imu_msg.payload, imu_msg.len );
}

// inceptors
void message_link_t::write_inceptors() {
    nst_message::inceptors_v2_t &inceptor_msg = comms_mgr->packer.inceptor_msg;
    send_packet( inceptor_msg.id, inceptor_msg.payload, inceptor_msg.len);
}

// fcs outputs
void message_link_t::write_outputs() {
    nst_message::fcs_outputs_v1_t &outputs_msg = comms_mgr->packer.outputs_msg;
    send_packet( outputs_msg.id, outputs_msg.payload, outputs_msg.len);
}

// nav (ekf) data
void message_link_t::write_nav() {
    nst_message::nav_v6_t &nav_msg = comms_mgr->packer.nav_msg;
    send_packet( nav_msg.id, nav_msg.payload, nav_msg.len );
}

// nav (ekf) metrics
void message_link_t::write_nav_metrics() {
    nst_message::nav_metrics_v6_t &metrics_msg = comms_mgr->packer.nav_metrics_msg;
    send_packet( metrics_msg.id, metrics_msg.payload, metrics_msg.len );
}

// fcs reference values
void message_link_t::write_refs() {
    nst_message::fcs_refs_v1_t &refs_msg = comms_mgr->packer.refs_msg;
    send_packet( refs_msg.id, refs_msg.payload, refs_msg.len );
}

// mission values
void message_link_t::write_mission() {
    nst_message::mission_v1_t &mission_msg = comms_mgr->packer.mission_msg;
    if ( mission_msg.millis > mission_last_millis ) {
        mission_last_millis = mission_msg.millis;
        send_packet( mission_msg.id, mission_msg.payload, mission_msg.len );
    }
}

void message_link_t::write_power() {
    nst_message::power_v2_t &power_msg = comms_mgr->packer.power_msg;
    send_packet( power_msg.id, power_msg.payload, power_msg.len );
}

// system status
void message_link_t::write_status() {
    nst_message::status_v8_t &status_msg = comms_mgr->packer.status_msg;
    if ( status_msg.millis > status_last_millis ) {
        status_last_millis = status_msg.millis;
        send_packet( status_msg.id, status_msg.payload, status_msg.len );
    }
}

int message_link_t::send_packet(uint8_t packet_id, uint8_t *payload, uint16_t len) {
    if ( serial_buffer.size() < max_buf_size - (len + 7) ) {
        // start of message sync (2) bytes
        serial_buffer.push(SerialLink::START_OF_MSG0);
        serial_buffer.push(SerialLink::START_OF_MSG1);

        // packet id (1 byte)
        serial_buffer.push(packet_id);

        // packet length (2 bytes)
        uint8_t len_lo = len & 0xFF;
        uint8_t len_hi = len >> 8;
        serial_buffer.push(len_lo);
        serial_buffer.push(len_hi);

        // write payload
        for ( uint16_t i = 0; i < len; i++ ) {
            serial_buffer.push( payload[i] );
        }

        // check sum (2 bytes)
        uint8_t cksum0, cksum1;
        SerialLink::checksum( packet_id, len_lo, len_hi, payload, len, &cksum0, &cksum1 );
        serial_buffer.push(cksum0);
        serial_buffer.push(cksum1);

        if ( serial_buffer.size() > max_buffer_used ) {
            max_buffer_used = serial_buffer.size();
            comms_node.setUInt("serial_link_max_buffer_used", max_buffer_used, saved_port );
        }

        return len + 7;
    } else {
        buffer_overrun_count++;
        comms_node.setUInt("serial_buffer_overruns", buffer_overrun_count, saved_port );
        return 0;
    }
}
void message_link_t::write_bytes() {
    // printf("Emptying log buffer: %d\n", data_logger_t::log_buffer.size());
    int count = 0;
    uint8_t val;
    while ( not serial_buffer.isEmpty() and serial._port->availableForWrite() ) {
        serial_buffer.pop(val);
        serial._port->write(val);
        count++;
    }
}

// hil_testing_node values are set in config file
bool message_link_t::parse_message( uint8_t id, uint8_t *buf, uint8_t message_size ) {
    bool result = false;
    //printf("message id: %d  len: %d\n", id, message_size);
    if ( id == nst_message::command_v1_id ) {
        nst_message::command_v1_t msg;
        msg.unpack(buf, message_size);
        printf("received command: %s %d\n", msg.message.c_str(), msg.sequence_num);
        uint8_t command_result = 0;
        command_result = execute_command(&msg, &serial);
        write_ack( msg.sequence_num, command_result );
        comms_node.setDouble("last_command_sec", millis() / 1000.0);
        result = true;
    } else if ( id == nst_message::airdata_v9_id ) {
        if ( hil_testing_node.getBool("enable") ) {
            nst_message::airdata_v9_t msg;
            msg.unpack(buf, message_size);
            msg.msg2props(airdata_node);
            airdata_node.setUInt("millis", millis());
        }
    } else if ( id == nst_message::gps_v5_id ) {
        if ( hil_testing_node.getBool("enable") ) {
            nst_message::gps_v5_t msg;
            msg.unpack(buf, message_size);
            msg.msg2props(gps_node);
            gps_node.setUInt("millis", millis());
            gps_node.setBool("settle", true);
            gps_node.setDouble("timestamp", millis() / 1000.0); // fixme: can we adjust remaining code to not use this field?
            gps_node.setDouble("latitude_deg", (double)(gps_node.getInt("latitude_raw")) / 10000000.0l);
            gps_node.setDouble("longitude_deg", (double)(gps_node.getInt("longitude_raw")) / 10000000.0l);
            // gps_node.pretty_print();
        }
    } else if ( id == nst_message::imu_v6_id ) {
        if ( hil_testing_node.getBool("enable") ) {
            nst_message::imu_v6_t msg;
            msg.unpack(buf, message_size);
            msg.msg2props(imu_node);
            imu_node.setUInt("millis", millis()); // force our own timestamp
            imu_node.setDouble("timestamp", millis() / 1000.0); // fixme: can we adjust remaining code to not use this field?
            imu_node.setUInt("gyros_calibrated", 2);  // flag gyros from external source as calibrated
            // imu_node.pretty_print();
        }
    } else if ( id == nst_message::inceptors_v2_id ) {
        if ( hil_testing_node.getBool("enable") and hil_testing_node.getString("inceptors") != "rc" ) {
            nst_message::inceptors_v2_t msg;
            msg.unpack(buf, message_size);
            msg.msg2props(inceptors_node);
            inceptors_node.setUInt("millis", millis());
        }
    } else if ( id == nst_message::power_v2_id ) {
        if ( hil_testing_node.getBool("enable") ) {
            nst_message::power_v2_t msg;
            msg.unpack(buf, message_size);
            msg.msg2props(power_node);
            power_node.setUInt("millis", millis()); // force our own timestamp
        }
    } else {
        printf("unknown/unexpected message id: %d len: %d\n", id, message_size);
    }
    return result;
}

void message_link_t::read_commands() {
    while ( serial.update() ) {
        parse_message( serial.pkt_id, serial.payload, serial.pkt_len );
    }
}
