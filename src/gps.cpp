#include <Arduino.h>
#include <TimeLib.h>

#include "nav_common/constants.h"
#include "nav_common/coremag.h"
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
        update_unix_sec();
        if ( !gps_acquired and gps_data.fixType == 3 ) {
            // first 3d fix
            gps_acquired = true;
            gps_settle_timer = 0;
            update_magvar();
            Serial.println("GPS: 3d fix acquired.");
            Serial.print("GPS: unix time = ");
            Serial.println(unix_sec, 0);
            Serial.print("Local magvar (deg) = ");
            Serial.println(magvar_rad*R2D);
        }
    }
}

bool gps_t::settle() {
    if ( gps_acquired ) {
        return gps_settle_timer > 10000; // 10 seconds
    } else {
        return false;
    }
}

void gps_t::update_unix_sec() {
    tmElements_t tm;
    int yr = gps_data.year;
    if (yr > 99) {
        yr = yr - 1970;
    } else {
        yr += 30;
    }
    tm.Year = yr;
    tm.Month = gps_data.month;
    tm.Day = gps_data.day;
    tm.Hour = gps_data.hour;
    tm.Minute = gps_data.min;
    tm.Second = gps_data.sec;
    unix_sec = makeTime(tm);
}

void gps_t::update_magvar() {
    long int jd = unixdate_to_julian_days( unix_sec );
    Serial.print("GPS: julian days = ");
    Serial.println(jd);
    double lat_rad = (gps_data.lat / 10000000.0) * D2R;
    double lon_rad = (gps_data.lon / 10000000.0) * D2R;
    float alt_m = gps_data.hMSL / 1000.0;
    double fields[6];
    magvar_rad = calc_magvar( lat_rad, lon_rad, alt_m / 1000.0, jd, fields );
    mag_ned(0) = fields[3];
    mag_ned(1) = fields[4];
    mag_ned(2) = fields[5];
    mag_ned.normalize();
    Serial.print("GPS: ideal mag vector = ");
    Serial.print(mag_ned(0), 3); Serial.print(" ");
    Serial.print(mag_ned(1), 3); Serial.print(" ");
    Serial.print(mag_ned(2), 3); Serial.println();
}

// shared global instance
gps_t gps;
