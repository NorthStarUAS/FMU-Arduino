#pragma once

#include "serial_link.h"
#include "../util/ratelimiter.h"

class message_link_t {

public:

    message_link_t();
    ~message_link_t();

    // Serial = usb, Serial1 connects to /dev/ttyO4 on beaglebone in
    // aura-v2 and marmot-v1 hardware
    SerialLink serial;
    unsigned long output_counter = 0;

    void init(uint8_t port, uint32_t baud);
    void update();
    void read_commands();
    bool is_inited() { return saved_port >= 0; }

private:

    int saved_port = -1;
    uint32_t saved_baud = 0;
    uint32_t event_last_millis = 0;
    uint32_t gps_last_millis = 0;
    uint32_t mission_last_millis = 0;
    uint32_t status_last_millis = 0;
    // uint32_t bytes_last_millis = 0;
    // uint16_t route_counter = 0;

    int write_ack( uint16_t sequence_num, uint8_t result );
    int write_airdata();
    int write_refs();
    int write_mission();
    int write_inceptors();
    int write_effectors();
    int write_imu();
    int write_gps();
    int write_nav();
    int write_nav_metrics();
    int write_power();
    int write_status();
    int write_events();
    bool parse_message( uint8_t id, uint8_t *buf, uint8_t message_size );

    RateLimiter limiter_50hz;
    RateLimiter limiter_10hz;
    RateLimiter limiter_4hz;
    RateLimiter limiter_2_5hz;
    RateLimiter limiter_2hz;
    RateLimiter limiter_1sec;
    RateLimiter limiter_2sec;
    RateLimiter limiter_10sec;
};
