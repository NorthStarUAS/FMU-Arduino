#pragma once

#include <Arduino.h>
#include <string>
using std::string;

class myprofile {

public:

    myprofile( string prof_name );
    ~myprofile();

    void start();
    uint32_t stop();
    void print_stats( string preface = "" );
    void to_props();

    uint32_t count;
    elapsedMillis total_millis;
    elapsedMicros interval;
    uint32_t min_interval;
    uint32_t max_interval;
    double sum_time;
    uint32_t overruns;

private:

    string name;

};

extern myprofile main_prof;
extern myprofile sensors_prof;
extern myprofile fcs_prof;
extern myprofile nav_prof;
extern myprofile mission_prof;
extern myprofile comms_prof;

void profile_print_stats();
void profile_to_props();