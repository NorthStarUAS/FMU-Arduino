#include <Arduino.h>

#include "gps.h"

#include "sensors/UBLOX8/UBLOX8.h"
static UBLOX8 m8n(&Serial3); // ublox m8n

void gps_t::setup() {
    // initialize the gps receiver
    m8n.begin(115200);
}

void gps_t::update() {
    // suck in any available gps bytes
    if ( m8n.read_ublox8() ) {
        new_gps_data = true;
        // gps_data = gps.get_data();
        m8n.update_data(&gps_data, sizeof(gps_data));
    }
}

// shared global instance
gps_t gps;
