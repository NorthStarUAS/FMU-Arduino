#include <math.h>

#include "config.h"
#include "gps.h"
#include "imu_mgr.h"

#include "nav_mgr.h"

void ekf_t::setup() {
    #if defined(AURA_ONBOARD_EKF)
    Serial.print("EKF: available  Current setting: ");
    if ( config.ekf.select == message::enum_nav::none ) {
        Serial.println("none");
    } else if ( config.ekf.select == message::enum_nav::nav15 ) {
        Serial.println("15 state ins/gns");
    } else if ( config.ekf.select == message::enum_nav::nav15_mag ) {
        Serial.println("15 state ins/gns/mag");
    } else {
        Serial.println("unknown setting/disabled");
    }
    #else
    Serial.println("EKF: not available for Teensy 3.2");
    #endif
}

void ekf_t::update() {
    #if defined(AURA_ONBOARD_EKF)
    IMUdata imu1;
    imu1.time = imu.imu_millis / 1000.0;
    imu1.p = imu.get_p_cal();
    imu1.q = imu.get_q_cal();
    imu1.r = imu.get_r_cal();
    imu1.ax = imu.get_ax_cal();
    imu1.ay = imu.get_ay_cal();
    imu1.az = imu.get_az_cal();
    imu1.hx = imu.get_hx_cal();
    imu1.hy = imu.get_hy_cal();
    imu1.hz = imu.get_hz_cal();

    GPSdata gps1;
    gps1.time = imu.imu_millis / 1000.0;
    gps1.unix_sec = gps1.time;
    gps1.lat = gps.gps_data.lat / 10000000.0;
    gps1.lon = gps.gps_data.lon / 10000000.0;
    gps1.alt = gps.gps_data.hMSL / 1000.0;
    gps1.vn = gps.gps_data.velN / 1000.0;
    gps1.ve = gps.gps_data.velE / 1000.0;
    gps1.vd = gps.gps_data.velD / 1000.0;
    gps1.unix_sec = gps.unix_sec;

    if ( !ekf_inited and gps.settle() ) {
        if ( config.ekf.select == message::enum_nav::nav15 ) {
            ekf.init(imu1, gps1);
        } else if ( config.ekf.select == message::enum_nav::nav15_mag ) {
            ekf_mag.init(imu1, gps1);
        }
        ekf_inited = true;
        Serial.println("EKF: initialized");
    } else if ( ekf_inited ) {
        if ( config.ekf.select == message::enum_nav::nav15 ) {
            ekf.time_update(imu1);
        } else if ( config.ekf.select == message::enum_nav::nav15_mag ) {
            ekf_mag.time_update(imu1);
        }
        if ( gps.gps_millis > gps_last_millis ) {
            gps_last_millis = gps.gps_millis;
            if ( config.ekf.select == message::enum_nav::nav15 ) {
                ekf.measurement_update(gps1);
            } else if ( config.ekf.select == message::enum_nav::nav15_mag ) {
                ekf_mag.measurement_update(imu1, gps1);
            }
            status = 2;         // ok
        }
        if ( config.ekf.select == message::enum_nav::nav15 ) {
            nav = ekf.get_nav();
        } else if ( config.ekf.select == message::enum_nav::nav15_mag ) {
            nav = ekf_mag.get_nav();
        }

        // sanity checks in case degenerate input leads to the filter
        // blowing up.  look for nans (or even negative #'s) in the
        // covariance matrix.
        if ( std::isnan(nav.Pp0) or std::isnan(nav.Pv0) or std::isnan(nav.Pa0)
             or (nav.Pp0 < -0.1) or (nav.Pv0 < -0.1) or (nav.Pa0 < -0.1) ) {
            Serial.println("filter blew up...");
            status = 0;
            reinit();
        }
        if ( millis() > (gps_last_millis + 2000) ) {
            // last gps message > 2 seconds ago
            status = 1;         // no gps
        }
    } else {
        status = 0;             // not initialized
    }
    #endif // AURA_ONBOARD_EKF
}

void ekf_t::reinit() {
    ekf_inited = false;
}
// global shared instance
ekf_t ekf;
