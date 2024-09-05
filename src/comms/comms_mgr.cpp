#include <Arduino.h>

#include "../nodes.h"
#include "../props2.h"
#include "info.h"
#include "comms_mgr.h"

// fixme: move lost link detection over to here!  The lost link task should just
// respond to the link state, not determine it.

void comms_mgr_t::init() {
    PropertyNode config_comms_node = PropertyNode("/config/comms");

    if ( comms_node.hasChild("lost_link_timeout_sec") ) {
        lost_link_timeout_sec = comms_node.getDouble("lost_link_timeout_sec");
    }
    comms_node.setBool("link_state", true);  // start optimistic

    status = RateLimiter(0.1);

    if ( config_comms_node.hasChild("gcs") ) {
        PropertyNode gcs_node = config_comms_node.getChild("gcs");
        int port = gcs_node.getUInt("port");
        int baud = gcs_node.getUInt("baud");
        if ( port > 0 and baud > 0 ) {
            gcs_link.init(port, baud /*, "gcs"*/);
        } else {
            printf("comms config error in gcs link section!\n");
            delay(500);
        }
    } else {
        printf("No gcs comms link configured.\n");
    }

    if ( config_comms_node.hasChild("host") ) {
        PropertyNode host_node = config_comms_node.getChild("host");
        int port = host_node.getUInt("port");
        int baud = host_node.getUInt("baud");
        if ( port > 0 and baud > 0 ) {
            host_link.init(port, baud /*, "host"*/);
        } else {
            printf("comms config error in host link section!\n");
            delay(500);
        }
    } else {
        printf("No host comms link configured.\n");
    }

    console.init();
    data_logger.init(data_logger_t::MID_RATE);

    event_mgr = new events_t();
    event_mgr->clear_events();

    delay(100);
}

void comms_mgr_t::update() {
    // lost link detection: message_link.cpp records the last_command_sec value
    // whenever a remote command is received.  We expect at least a hb command
    // every 10 seconds. If too much time has elapsed since the last received
    // command we flag a lost link state here.  The mission manager is
    // responsible for deciding what action to take if this flag is set or
    // cleared.
    float last_command_sec = comms_node.getDouble("last_command_sec");
    // // simulate lost link for testing, but don't carry this code into a final build
    // if ( comms_node.getBool("simulate_lost_link") ) {
    //     last_command_sec = 0.0;
    // }
    float current_sec = millis() / 1000.0;
    if ( current_sec - last_command_sec < lost_link_timeout_sec ) {
        // link ok
        if ( not comms_node.getBool("link_state") ) {
            event_mgr->add_event("comms", "link ok");
        }
        comms_node.setBool("link_state", true);
    } else {
        // it has been too long, link lost
        if ( comms_node.getBool("link_state") ) {
            event_mgr->add_event("comms", "link lost");
        }
        comms_node.setBool("link_state", false);
    }

    // build all the messages for comms and logging
    packer.update();

    if ( gcs_link.is_inited() ) {
        gcs_link.read_commands();
        gcs_link.write_messages();
        uint32_t bytes = comms_node.getUInt("bytes_to_gcs");
        bytes += gcs_link.output_counter;
        comms_node.setUInt("bytes_to_gcs", bytes);
        gcs_link.output_counter = 0;
    }

    if ( host_link.is_inited() ) {
        host_link.read_commands();
	    host_link.write_messages();
    }

    // human console interaction begins when gyros finish calibrating
    // if ( imu_node.getUInt("gyros_calibrated") != 2 ) {
    //     return;
    // }

    console.update();

    data_logger.log_messages();

    // the link objects will each write all accumulated events each frame, so
    // then we clear them here after all the links have updated.
    event_mgr->clear_events();

    // 10 second heartbeat console output
    if ( status.update() ) {
        write_status_info_ascii();
        write_power_ascii();
        // float elapsed_sec = (millis() - tempTimer) / 1000.0;
        printf("Available mem: %u bytes\n", status_node.getUInt("available_memory"));
        // Serial.print("Performance = ");
        // Serial.print(counter/elapsed_sec, 1);
        // Serial.println(" hz");
        printf("\n");
    }
}

comms_mgr_t *comms_mgr = nullptr;