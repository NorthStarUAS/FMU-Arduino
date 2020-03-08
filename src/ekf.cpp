#include "gps.h"
#include "imu.h"

#include "ekf.h"

void ekf_t::update() {
    // handle ekf init/update
    if ( !gps_found and gps.new_gps_data ) {
        gps_found = true;
        gpsSettle = 0;
        Serial.println("EKF: gps found itself");
    }
    
    IMUdata imu1;
    imu1.time = imu.imu_micros / 1000000.0;
    imu1.p = imu.get_p();
    imu1.q = imu.get_q();
    imu1.r = imu.get_r();
    imu1.ax = imu.get_ax();
    imu1.ay = imu.get_ay();
    imu1.az = imu.get_az();
    imu1.hx = imu.get_hx();
    imu1.hy = imu.get_hy();
    imu1.hz = imu.get_hz();
    
    GPSdata gps1;
    gps1.time = imu.imu_micros / 1000000.0;
    gps1.unix_sec = gps1.time;
    gps1.lat = gps.gps_data.lat / 10000000.0;
    gps1.lon = gps.gps_data.lon / 10000000.0;
    gps1.alt = gps.gps_data.hMSL / 1000.0;
    gps1.vn = gps.gps_data.velN / 1000.0;
    gps1.ve = gps.gps_data.velE / 1000.0;
    gps1.vd = gps.gps_data.velD / 1000.0;
    
    if ( !ekf_inited and gps_found and gpsSettle > 10000 ) {
        ekf.init(imu1, gps1);
        ekf_inited = true;
        Serial.println("EKF: initialized");
    } else if ( ekf_inited ) {
        ekf.time_update(imu1);
        if ( gps.new_gps_data ) {
            ekf.measurement_update(gps1);
        }
        nav = ekf.get_nav();
    }
}

// global shared instance
ekf_t ekf;
