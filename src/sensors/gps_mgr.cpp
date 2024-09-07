#include <Arduino.h>
#include <TimeLib.h>

#include "../nodes.h"

#include "../util/constants.h"
#include "../nav/coremag.h"
#include "gps_mgr.h"

#include "../sensors/UBLOX8/UBLOX8.h"
UBLOX8 m8n(&Serial3); // ublox m8n

void gps_mgr_t::init() {
    gps_node.setBool("settle", false);

    // initialize the gps receiver
    m8n.begin(115200);
}

void gps_mgr_t::update() {
    // suck in any available gps bytes
    if ( m8n.read_ublox8() ) {
        m8n.update_data(&gps_data, sizeof(gps_data));
        gps_millis = millis();
        update_unix_usec();
        if ( gps_data.fixType >= 3 ) {
            if ( !gps_acquired ) {
                // first 3d fix
                gps_acquired = true;
                gps_settle_timer = 0;
                update_magvar();
                Serial.println("GPS: 3d fix acquired.");
                Serial.print("GPS: unix time = "); Serial.println((double)unix_usec / 1000000.0, 3);
                Serial.print("Local magvar (deg) = "); Serial.println(magvar_rad*r2d, 2);
            } else if ( !gps_settled and gps_settle_timer > 10000 ) {  // 10 seconds
                printf("GPS: settled for 10 seconds.\n");
                gps_settled = true;
                gps_node.setBool("settle", true);
            }
        } else {
            if ( gps_acquired and !gps_settled ) {
                // unaquire if we lose fix before settling
                gps_acquired = false;
                printf("lost fix before settling, unaquire gps\n");
            }
        }

        // publish
        gps_node.setUInt("millis", gps_millis);
        gps_node.setDouble("timestamp", gps_millis / 1000.0);
        gps_node.setUInt64("unix_usec",  unix_usec);
        gps_node.setInt("latitude_raw", gps_data.lat);
        gps_node.setInt("longitude_raw", gps_data.lon);
        gps_node.setDouble("latitude_deg", (double)(gps_data.lat) / 10000000.0l);
        gps_node.setDouble("longitude_deg", (double)(gps_data.lon) / 10000000.0l);
        gps_node.setDouble("altitude_m", gps_data.height / 1000.0);
        gps_node.setDouble("alt_msl_m", gps_data.hMSL / 1000.0);
        gps_node.setDouble("vn_mps", gps_data.velN / 1000.0);
        gps_node.setDouble("ve_mps", gps_data.velE / 1000.0);
        gps_node.setDouble("vd_mps", gps_data.velD / 1000.0);
        gps_node.setInt("num_sats", gps_data.numSV);
        gps_node.setInt("status", gps_data.fixType);
        gps_node.setDouble("hAcc_m", gps_data.hAcc / 1000.0);
        gps_node.setDouble("vAcc_m", gps_data.vAcc / 1000.0);
        gps_node.setDouble("pdop", gps_data.pDOP);
        gps_node.setInt("year", gps_data.year);
        gps_node.setInt("month", gps_data.month);
        gps_node.setInt("day", gps_data.day);
        gps_node.setInt("hour", gps_data.hour);
        gps_node.setInt("min", gps_data.min);
        gps_node.setInt("sec", gps_data.sec);
        // gps_node.pretty_print();
    }
}

void gps_mgr_t::update_unix_usec() {
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
    unix_usec = (uint64_t)makeTime(tm) * 1000000;
    unix_usec += gps_data.nano / 1000;
}

void gps_mgr_t::update_magvar() {
    long int jd = unixdate_to_julian_days( unix_usec / 1000000 );
    Serial.print("GPS: julian days = "); Serial.println(jd);
    double lat_rad = (gps_data.lat / 10000000.0) * d2r;
    double lon_rad = (gps_data.lon / 10000000.0) * d2r;
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
    Serial.print(mag_ned(2), 3); Serial.println("");
}