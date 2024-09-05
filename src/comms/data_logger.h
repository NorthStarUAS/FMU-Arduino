#pragma once

#include "../logs/RingBuf.h"

#include "../util/ratelimiter.h"

static const unsigned int max_buf_size = 1024;

class data_logger_t {

public:

    static RingBuf<uint8_t, max_buf_size> log_buffer;
    unsigned long output_counter = 0;

    enum log_rate_t { HIGH_RATE, MID_RATE, LOW_RATE };
    void init(log_rate_t rate);
    void log_messages();

private:

    log_rate_t log_rate;
    uint32_t event_last_millis = 0;
    uint32_t gps_last_millis = 0;
    uint32_t mission_last_millis = 0;
    uint32_t status_last_millis = 0;

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

    RateLimiter limiter_50hz;
    RateLimiter limiter_25hz;
    RateLimiter limiter_10hz;
    RateLimiter limiter_1sec;
    RateLimiter limiter_2sec;
    RateLimiter limiter_10sec;

    int log_packet(uint8_t packet_id, uint8_t *payload, uint16_t len);

};
