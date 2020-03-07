#pragma once

#include "util/serial_link.h"
#include "sensors/UBLOX8/UBLOX8.h"

class comms_t {
private:
    float autopilot_norm[SBUS_CHANNELS];
    
public:
    // Serial = usb, Serial1 connects to /dev/ttyO4 on beaglebone in
    // aura-v2 and marmot-v1 hardware
    SerialLink serial;
    unsigned long output_counter = 0;

    void setup();
    int write_ack_bin( uint8_t command_id, uint8_t subcommand_id );
    int write_pilot_in_bin();
    int write_imu_bin();
    int write_gps_bin(ublox8_nav_pvt_t *gps_data, bool new_gps_data);
    void write_gps_ascii(ublox8_nav_pvt_t *gps_data);
    int write_airdata_bin();
    int write_power_bin();
    int write_status_info_bin();
    bool parse_message_bin( byte id, byte *buf, byte message_size );
    void read_commands();
};

extern comms_t comms;
