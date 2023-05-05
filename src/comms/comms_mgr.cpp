#include "comms_mgr.h"

void comms_mgr_t::init() {
    config_node = PropertyNode("/config/comms");
    imu_node = PropertyNode("/sensors/imu");
    status_node = PropertyNode("/status");
    
    info_timer = RateLimiter(10);
    heartbeat = RateLimiter(0.1);
    tempTimer = AP_HAL::millis();
    counter = 0;

    if ( config_node.hasChild("gcs") ) {
        PropertyNode gcs_node = config_node.getChild("gcs");
        int port = gcs_node.getUInt("port");
        int baud = gcs_node.getUInt("baud");
        if ( port > 0 and baud > 0 ) {
            gcs_link.init(port, baud, "gcs");
        } else {
            printf("comms config error in gcs link section!\n");
            hal.scheduler->delay(500);
        }
    } else {
        printf("No gcs comms link configured.\n");
    }
    if ( config_node.hasChild("host") ) {
        PropertyNode host_node = config_node.getChild("host");
        int port = host_node.getUInt("port");
        int baud = host_node.getUInt("baud");
        if ( port > 0 and baud > 0 ) {
            host_link.init(port, baud, "host");
        } else {
            printf("comms config error in host link section!\n");
            hal.scheduler->delay(500);
        }
    } else {
        printf("No host comms link configured.\n");
    }
    
    info.init();

    menu.init();
    
    hal.scheduler->delay(100);
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
    if ( imu_node.getUInt("gyros_calibrated") != 2 ) {
        return;
    }

    if ( info_timer.update() ) {
        menu.update();
        if ( menu.display_pilot ) { info.write_pilot_in_ascii(); }
        if ( menu.display_gps ) { info.write_gps_ascii(); }
        if ( menu.display_airdata ) { info.write_airdata_ascii(); }
        if ( menu.display_imu ) { info.write_imu_ascii(); }
        if ( menu.display_nav ) { info.write_nav_ascii(); }
        if ( menu.display_nav_stats ) { info.write_nav_stats_ascii(); }
        if ( menu.display_act ) { info.write_actuator_out_ascii(); }
    }

    // 10 second heartbeat console output
    if ( heartbeat.update() ) {
        info.write_status_info_ascii();
        info.write_power_ascii();
        float elapsed_sec = (AP_HAL::millis() - tempTimer) / 1000.0;
        printf("Available mem: %d bytes\n",
               status_node.getUInt("available_memory"));
        printf("Performance = %.1f hz\n", counter/elapsed_sec);
        //PropertyNode("/").pretty_print();
        printf("\n");

#if 0
        // system info
        ExpandingString dma_info {};
        ExpandingString mem_info {};
        ExpandingString uart_info {};
        ExpandingString thread_info {};
        hal.util->dma_info(dma_info);
        hal.util->mem_info(mem_info);
        hal.util->uart_info(uart_info);
        hal.util->thread_info(thread_info);
        console->printf("dma info:\n%s\n", dma_info.get_string());
        console->printf("mem info:\n%s\n", mem_info.get_string());
        console->printf("uart info:\n%s\n", uart_info.get_string());
        // console->printf("thread info:\n%s\n", thread_info.get_string());
#endif
    }
}
