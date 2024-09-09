#include "../nodes.h"
#include "profile.h"

#include <Arduino.h>

myprofile::myprofile( string prof_name ) {
    name = prof_name;
    total_millis = 0;
    count = 0;
    sum_time = 0;
    max_interval = 0;
    min_interval = 10000;
    overruns = 0;
}

myprofile::~myprofile() {
}

void myprofile::start() {
    if ( count == 0 ) {
	    total_millis = 0;
    }
    interval = 0;
    count++;
}

uint32_t myprofile::stop() {
    uint32_t elapsed = interval;
    sum_time += elapsed;

    // log situations where a module took longer that 0.10 sec to execute
    // if ( elapsed > 0.10 ) {
    //     char msg[256];
    //     snprintf(msg, 256, "t1 = %.3f t2 = %.3f int = %.3f",
    //     	 start_time, stop_time, elapsed);
    //     events->log( name.c_str(), msg );
    // }

    if ( elapsed < min_interval ) {
	    min_interval = elapsed;
    }
    if ( elapsed > max_interval ) {
	    max_interval = elapsed;
    }
    if ( elapsed > 40000 ) {
        overruns++;
    }
    return elapsed;
}

void myprofile::print_stats( string preface ) {
    float avg_hz = 0.0;
    if ( total_millis > 1 ) {
    	avg_hz = (float)count * 1000 / total_millis;
    }
    Serial.print(preface.c_str());
    Serial.print(name.c_str());
    Serial.print(" avg: ");
    Serial.print((sum_time/1000.0) / (float)count, 2);
    Serial.print("(us) num: ");
    Serial.print(count);
    Serial.print(" tot: ");
    Serial.print(sum_time/1000000.0, 2);
    Serial.print("(s) (range: ");
    Serial.print(min_interval/1000.0, 2);
    Serial.print("-");
    Serial.print(max_interval/1000.0, 2);
    Serial.print(") hz: ");
    Serial.print(avg_hz, 1);
    if ( overruns > 0 ) {
        Serial.print(" over: ");
        Serial.print(overruns);
    }
    Serial.println();
}

void myprofile::to_props() {
    PropertyNode node = profile_node.getChild(name.c_str());
    float avg_hz = 0.0;
    if ( total_millis > 1 ) {
    	avg_hz = (float)count * 1000 / total_millis;
    }
    node.setDouble("avg_us", (sum_time/1000.0) / (float)count);
    node.setDouble("min_us", min_interval/1000.0);
    node.setDouble("max_us", max_interval/1000.0);
    node.setDouble("avg_hz", avg_hz);
    node.setUInt("overruns", overruns);
}

myprofile main_prof("main_loop");
myprofile sensors_prof("sensors");
myprofile fcs_prof("fcs");
myprofile nav_prof("ekf");
myprofile mission_prof("mission");

void profile_print_stats() {
    main_prof.print_stats();
    sensors_prof.print_stats();
    fcs_prof.print_stats();
    nav_prof.print_stats();
    mission_prof.print_stats();
}

void profile_to_props() {
    main_prof.to_props();
    sensors_prof.to_props();
    fcs_prof.to_props();
    nav_prof.to_props();
    mission_prof.to_props();
}