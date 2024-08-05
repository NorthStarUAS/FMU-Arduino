#pragma once

#include <Arduino.h>

// #include <inttypes.h>

class myprofile {

public:
    myprofile();
    ~myprofile();

    void set_name( const char *_name );
    void start();
    uint32_t stop();
    void stats( const char *preface );
    // inline uint32_t get_last_interval() { return last_interval; }

private:
    uint32_t count;
    elapsedMillis total_millis;
    elapsedMicros interval;
    uint32_t min_interval;
    uint32_t max_interval;
    double sum_time;
    uint32_t overruns;
    char name[17];
};
