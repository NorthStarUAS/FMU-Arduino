#pragma once

#include "util/serial_link.h"
#include "sensors/sbus/sbus.h"
#include "sensors/UBLOX8/UBLOX8.h"

class comms_t {
public:
    // Serial = usb, Serial1 connects to /dev/ttyO4 on beaglebone in
    // aura-v2 and marmot-v1 hardware
    SerialLink serial;
    unsigned long output_counter = 0;
    int main_loop_timer_misses = 0; // performance sanity check

    void setup();
    int write_ack_bin( uint8_t command_id, uint8_t subcommand_id );
    int write_pilot_in_bin();
    int write_imu_bin();
    int write_gps_bin();
    void write_gps_ascii();
    int write_nav_bin();
    void write_nav_ascii();
    int write_airdata_bin();
    int write_power_bin();
    void write_power_ascii();
    int write_status_info_bin();
    void write_status_info_ascii();
    bool parse_message_bin( byte id, byte *buf, byte message_size );
    void read_commands();
    
private:
    unsigned long int gps_last_millis = 0;
};

extern comms_t comms;
