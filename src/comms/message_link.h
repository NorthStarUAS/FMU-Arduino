#pragma once

#include "serial_link.h"
#include "../logs/RingBuf.h"
#include "../util/ratelimiter.h"

class message_link_t {

public:

    // Serial = usb, Serial1 connects to /dev/ttyO4 on beaglebone in
    // aura-v2 and marmot-v1 hardware
    SerialLink serial;
    unsigned long output_counter = 0;

    void init(uint8_t port, uint32_t baud);
    void write_messages();
    void read_commands();
    bool is_inited() { return saved_port >= 0; }

private:

    static const unsigned int max_buf_size = 2048;
    RingBuf<uint8_t, max_buf_size> serial_buffer;

    int saved_port = -1;
    uint32_t saved_baud = 0;
    uint32_t event_last_millis = 0;
    uint32_t gps_last_millis = 0;
    uint32_t mission_last_millis = 0;
    uint32_t status_last_millis = 0;
    uint32_t max_buffer_used = 0;
    uint32_t buffer_overrun_count = 0;

    void write_ack( uint16_t sequence_num, uint8_t result );
    void write_airdata();
    void write_environment();
    void write_refs();
    void write_mission();
    void write_inceptors();
    void write_effectors();
    void write_imu();
    void write_gps();
    void write_nav();
    void write_nav_metrics();
    void write_power();
    void write_status();
    void write_events();
    bool parse_message( uint8_t id, uint8_t *buf, uint8_t message_size );

    int send_packet(uint8_t packet_id, uint8_t *payload, uint16_t len);
    void write_bytes();

    RateLimiter limiter_50hz;
    RateLimiter limiter_10hz;
    RateLimiter limiter_4hz;
    RateLimiter limiter_2_5hz;
    RateLimiter limiter_2hz;
    RateLimiter limiter_1sec;
    RateLimiter limiter_2sec;
    RateLimiter limiter_10sec;
};
