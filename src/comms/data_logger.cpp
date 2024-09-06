/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include <Arduino.h>
#include <MTP_Teensy.h>
#include <SD.h>

#include "../nodes.h"
#include "../ns_messages.h"

#include "../mission/mission_mgr.h"
#include "../nav/nav_constants.h"

#include "comms_mgr.h"
#include "serial_link.h"
#include "data_logger.h"

void data_logger_t::init(log_rate_t rate) {
    log_rate = rate;
    limiter_50hz = RateLimiter(50);
    limiter_25hz = RateLimiter(25);
    limiter_10hz = RateLimiter(10);
    limiter_1sec = RateLimiter(1);
    limiter_2sec = RateLimiter(0.5);
    limiter_10sec = RateLimiter(0.1);

    // initialize SD card
    if ( !SD.begin(BUILTIN_SDCARD)) {
        printf("Cannot initializing builtin SD card ... no card?\n");
    } else {
        printf("SD card initialized for logging.\n");
        sd_card_inited = true;
        MTP.addFilesystem(SD, "SD Card");
    }
}

void data_logger_t::log_messages() {
    // fixme: make externally configurable?
    if ( log_rate == HIGH_RATE ) {
        output_counter += write_events();
        if ( limiter_50hz.update() ) {
            output_counter += write_nav();
            output_counter += write_effectors();
            output_counter += write_imu();
            output_counter += write_inceptors();
            output_counter += write_gps();
            output_counter += write_airdata();
            output_counter += write_refs();
            output_counter += write_mission();
        }
        if ( limiter_1sec.update() ) {
            output_counter += write_power();
        }
        if ( limiter_2sec.update() ) {
            output_counter += write_nav_metrics();
        }
        if ( limiter_10sec.update() ) {
            output_counter += write_status();
        }
    } else if ( log_rate == MID_RATE ) {
        output_counter += write_events();
        if ( limiter_25hz.update() ) {
            output_counter += write_nav();
            output_counter += write_effectors();
            output_counter += write_imu();
            output_counter += write_inceptors();
            output_counter += write_gps();
            output_counter += write_airdata();
            output_counter += write_refs();
            output_counter += write_mission();
        }
        if ( limiter_1sec.update() ) {
            output_counter += write_power();
        }
        if ( limiter_2sec.update() ) {
            output_counter += write_nav_metrics();
        }
        if ( limiter_10sec.update() ) {
            output_counter += write_status();
        }
    } else if ( log_rate == LOW_RATE ) {
        output_counter += write_events();
        if ( limiter_10hz.update() ) {
            output_counter += write_nav();
            output_counter += write_effectors();
            output_counter += write_imu();
            output_counter += write_inceptors();
            output_counter += write_gps();
            output_counter += write_airdata();
            output_counter += write_refs();
            output_counter += write_mission();
        }
        if ( limiter_1sec.update() ) {
            output_counter += write_power();
        }
        if ( limiter_2sec.update() ) {
            output_counter += write_nav_metrics();
        }
        if ( limiter_10sec.update() ) {
            output_counter += write_status();
        }
    }
}

int data_logger_t::write_airdata() {
    ns_message::airdata_v8_t &air_msg = comms_mgr->packer.air_msg;
    return log_packet( air_msg.id, air_msg.payload, air_msg.len );
}

// final effector commands
int data_logger_t::write_effectors() {
    ns_message::effectors_v1_t &eff_msg = comms_mgr->packer.eff_msg;
    return log_packet( eff_msg.id, eff_msg.payload, eff_msg.len);
}

int data_logger_t::write_events() {
    ns_message::event_v3_t &event_msg = comms_mgr->packer.event_msg;
    if ( event_msg.millis > event_last_millis ) {
        event_last_millis = event_msg.millis;
        return log_packet( event_msg.id, event_msg.payload, event_msg.len );
    } else {
        return 0;
    }
}

int data_logger_t::write_gps() {
    ns_message::gps_v5_t &gps_msg = comms_mgr->packer.gps_msg;
    if ( gps_msg.millis > gps_last_millis ) {
        gps_last_millis = gps_msg.millis;
        return log_packet( gps_msg.id, gps_msg.payload, gps_msg.len );
    } else {
        return 0;
    }
}

int data_logger_t::write_imu() {
    ns_message::imu_v6_t &imu_msg = comms_mgr->packer.imu_msg;
    return log_packet( imu_msg.id, imu_msg.payload, imu_msg.len );
}

// inceptors
int data_logger_t::write_inceptors() {
    ns_message::inceptors_v2_t &inceptor_msg = comms_mgr->packer.inceptor_msg;
    return log_packet( inceptor_msg.id, inceptor_msg.payload, inceptor_msg.len);
}

// nav (ekf) data
int data_logger_t::write_nav() {
    ns_message::nav_v6_t &nav_msg = comms_mgr->packer.nav_msg;
    return log_packet( nav_msg.id, nav_msg.payload, nav_msg.len );
}

// nav (ekf) metrics
int data_logger_t::write_nav_metrics() {
    ns_message::nav_metrics_v6_t &metrics_msg = comms_mgr->packer.nav_metrics_msg;
    return log_packet( metrics_msg.id, metrics_msg.payload, metrics_msg.len );
}

// fcs reference values
int data_logger_t::write_refs() {
    ns_message::fcs_refs_v1_t &refs_msg = comms_mgr->packer.refs_msg;
    return log_packet( refs_msg.id, refs_msg.payload, refs_msg.len );
}

// mission values
int data_logger_t::write_mission() {
    ns_message::mission_v1_t &mission_msg = comms_mgr->packer.mission_msg;
    if ( mission_msg.millis > mission_last_millis ) {
        mission_last_millis = mission_msg.millis;
        return log_packet( mission_msg.id, mission_msg.payload, mission_msg.len );
    } else {
        return 0;
    }
}

int data_logger_t::write_power() {
    ns_message::power_v1_t &power_msg = comms_mgr->packer.power_msg;
    return log_packet( power_msg.id, power_msg.payload, power_msg.len );
}

// system status
int data_logger_t::write_status() {
    ns_message::status_v7_t &status_msg = comms_mgr->packer.status_msg;
    if ( status_msg.millis > status_last_millis ) {
        status_last_millis = status_msg.millis;
        return log_packet( status_msg.id, status_msg.payload, status_msg.len );
    } else {
        return 0;
    }
}

int data_logger_t::log_packet(uint8_t packet_id, uint8_t *payload, uint16_t len) {
    if ( log_buffer.size() < max_buf_size - (len + 7) ) {
        // start of message sync (2) bytes
        log_buffer.push(SerialLink::START_OF_MSG0);
        log_buffer.push(SerialLink::START_OF_MSG1);

        // packet id (1 byte)
        log_buffer.push(packet_id);

        // packet length (2 bytes)
        uint8_t len_lo = len & 0xFF;
        uint8_t len_hi = len >> 8;
        log_buffer.push(len_lo);
        log_buffer.push(len_hi);

        // write payload
        for ( uint16_t i = 0; i < len; i++ ) {
            log_buffer.push( payload[i] );
        }

        // check sum (2 bytes)
        uint8_t cksum0, cksum1;
        SerialLink::checksum( packet_id, len_lo, len_hi, payload, len, &cksum0, &cksum1 );
        log_buffer.push(cksum0);
        log_buffer.push(cksum1);

        return len + 7;
    } else {
        return 0;
    }
}

void data_logger_t::write_buffer() {
    // printf("Emptying log buffer: %d\n", data_logger_t::log_buffer.size());
    uint8_t val;
    while ( not data_logger_t::log_buffer.isEmpty() ) {
        data_logger_t::log_buffer.lockedPop(val);
    }
}

RingBuf<uint8_t, max_buf_size> data_logger_t::log_buffer;