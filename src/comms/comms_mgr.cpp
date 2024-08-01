#include <Arduino.h>

#include "../nodes.h"
#include "info.h"
#include "comms_mgr.h"

void comms_mgr_t::init() {
    config_comms_node = PropertyNode("/config/comms");

    heartbeat = RateLimiter(0.1);
    tempTimer = millis(); // fixme use ellapsedmillis?
    counter = 0;

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

    events = new events_t();
    events->clear_events();

    delay(100);
}

void comms_mgr_t::update() {
    counter++;

    if ( gcs_link.is_inited() ) {
        gcs_link.read_commands();
        gcs_link.update();
    }

    if ( host_link.is_inited() ) {
        host_link.read_commands();
	    host_link.update();
    }

    // human console interaction begins when gyros finish calibrating
    // if ( imu_node.getUInt("gyros_calibrated") != 2 ) {
    //     return;
    // }

    console.update();

    // the link objects will each write all accumulated events each frame, so
    // then we clear them here after all the links have updated.
    events->clear_events();

    // 10 second heartbeat console output
    if ( heartbeat.update() ) {
        write_status_info_ascii();
        write_power_ascii();
        float elapsed_sec = (millis() - tempTimer) / 1000.0;
        printf("Available mem: %u bytes\n", status_node.getUInt("available_memory"));
        Serial.print("Performance = ");
        Serial.print(counter/elapsed_sec, 1);
        Serial.println(" hz");
        printf("\n");
    }
}
