#pragma once

#include "../props2.h"
#include "serial.h"
#include "../util/ratelimiter.h"

class message_link_t {

public:

    message_link_t();
    ~message_link_t();

    // Serial = usb, Serial1 connects to /dev/ttyO4 on beaglebone in
    // aura-v2 and marmot-v1 hardware
    SerialLink serial;
    unsigned long output_counter = 0;
    string relay_id;

    void init(uint8_t port, uint32_t baud, string relay_name);
    void update();
    void read_commands();
    bool is_inited() { return saved_port >= 0; }

private:

    PropertyNode config_nav_node;
    PropertyNode nav_node;
    PropertyNode airdata_node;
    PropertyNode ap_node;
    PropertyNode circle_node;
    PropertyNode effectors_node;
    PropertyNode gps_node;
    PropertyNode home_node;
    PropertyNode imu_node;
    PropertyNode pilot_node;
    PropertyNode pos_node;
    PropertyNode power_node;
    PropertyNode route_node;
    PropertyNode status_node;
    PropertyNode switches_node;
    PropertyNode targets_node;
    PropertyNode task_node;

    int saved_port = -1;
    uint32_t gps_last_millis = 0;
    uint32_t bytes_last_millis = 0;
    uint16_t last_command_seq_num = 0;

    int write_ack( uint16_t sequence_num, uint8_t result );
    int write_airdata();
    int write_ap();
    int write_effectors();
    int write_imu();
    int write_gps();
    int write_nav();
    int write_nav_metrics();
    int write_pilot();
    int write_power();
    int write_status();
    bool parse_message( uint8_t id, uint8_t *buf, uint8_t message_size );

    RateLimiter airdata_limiter;
    RateLimiter ap_limiter;
    RateLimiter eff_limiter;
    RateLimiter gps_limiter;
    RateLimiter imu_limiter;
    RateLimiter mission_limiter;
    RateLimiter nav_limiter;
    RateLimiter nav_metrics_limiter;
    RateLimiter pilot_limiter;
    RateLimiter power_limiter;
    RateLimiter status_limiter;

};
