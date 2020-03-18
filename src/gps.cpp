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
        m8n.update_data(&gps_data, sizeof(gps_data));
        gps_millis = millis();
        if ( !gps_acquired and gps_data.fixType == 3 ) {
            // first 3d fix
            gps_acquired = true;
            gps_alive = 0;
            Serial.println("GPS: 3d fix acquired.");
        }
    }
}

bool gps_t::settle() {
    if ( gps_acquired ) {
        return gps_alive > 10000; // 10 seconds
    } else {
        return false;
    }
}

// shared global instance
gps_t gps;
