#pragma once

#include <Arduino.h>

class myprofile {

public:

    myprofile();
    ~myprofile();

    void set_name( const char *_name );
    void start();
    uint32_t stop();
    void print_stats( const char *preface );
    void to_props();

    uint32_t count;
    elapsedMillis total_millis;
    elapsedMicros interval;
    uint32_t min_interval;
    uint32_t max_interval;
    double sum_time;
    uint32_t overruns;

private:

    char name[17];

};
