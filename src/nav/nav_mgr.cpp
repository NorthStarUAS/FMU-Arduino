#include <Arduino.h>
#include <math.h>
#include <string>

#include "../nodes.h"
#include "../util/profile.h"

#include "nav_mgr.h"
#include "nav_constants.h"  // R2D

using std::string;

void nav_mgr_t::init() {
    config_nav_node = PropertyNode("/config/nav");

    string selected = config_nav_node.getString("select");
    // fix me ...
    if ( selected == ""  ) {
        selected = "none";
        config_nav_node.setString("select", selected);
    } else {
        selected = "nav15";         // force (fixme)
    }
    printf("EKF: selected: %s\n", selected.c_str());
    configure();
    printf("configured ekf:\n");
    config_nav_node.pretty_print();
}

void nav_mgr_t::configure() {
    string selected = config_nav_node.getString("select");
    NAVconfig config;
    if ( selected == "nav15" ) {
        config = ekf.get_config();
    } else if ( selected == "nav15_mag" ) {
        config = ekf_mag.get_config();
    }
    if ( config_nav_node.hasChild("sig_w_accel") ) {
        config.sig_w_ax = config_nav_node.getDouble("sig_w_accel");
        config.sig_w_ay = config.sig_w_ax;
        config.sig_w_az = config.sig_w_ax;
    }
    if ( config_nav_node.hasChild("sig_w_gyro") ) {
        config.sig_w_gx = config_nav_node.getDouble("sig_w_gyro");
        config.sig_w_gy = config.sig_w_gx;
        config.sig_w_gz = config.sig_w_gx;
    }
    if ( config_nav_node.hasChild("sig_a_d") ) {
        config.sig_a_d = config_nav_node.getDouble("sig_a_d");
    }
    if ( config_nav_node.hasChild("tau_a") ) {
        config.tau_a = config_nav_node.getDouble("tau_a");
    }
    if ( config_nav_node.hasChild("sig_g_d") ) {
        config.sig_g_d = config_nav_node.getDouble("sig_g_d");
    }
    if ( config_nav_node.hasChild("tau_g") ) {
        config.tau_g = config_nav_node.getDouble("tau_g");
    }
    if ( config_nav_node.hasChild("sig_gps_p_ne") ) {
        config.sig_gps_p_ne = config_nav_node.getDouble("sig_gps_p_ne");
    }
    if ( config_nav_node.hasChild("sig_gps_p_d") ) {
        config.sig_gps_p_d = config_nav_node.getDouble("sig_gps_p_d");
    }
    if ( config_nav_node.hasChild("sig_gps_v_ne") ) {
        config.sig_gps_v_ne = config_nav_node.getDouble("sig_gps_v_ne");
    }
    if ( config_nav_node.hasChild("sig_gps_v_d") ) {
        config.sig_gps_v_d = config_nav_node.getDouble("sig_gps_v_d");
    }
    if ( config_nav_node.hasChild("sig_mag") ) {
        config.sig_mag = config_nav_node.getDouble("sig_mag");
    }
    if ( selected == "nav15" ) {
        ekf.set_config(config);
    } else if ( selected == "nav15_mag" ) {
        ekf_mag.set_config(config);
    }
}

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

void nav_mgr_t::update() {
    nav_prof.start();

    IMUdata imu1;
    imu1.time_sec = imu_node.getDouble("timestamp");
    imu1.p_rps = imu_node.getDouble("p_rps");
    imu1.q_rps = imu_node.getDouble("q_rps");
    imu1.r_rps = imu_node.getDouble("r_rps");
    imu1.ax_mps2 = imu_node.getDouble("ax_mps2");
    imu1.ay_mps2 = imu_node.getDouble("ay_mps2");
    imu1.az_mps2 = imu_node.getDouble("az_mps2");
    imu1.hx = imu_node.getDouble("hx");
    imu1.hy = imu_node.getDouble("hy");
    imu1.hz = imu_node.getDouble("hz");

    GPSdata gps1;
    gps1.time_sec = gps_node.getDouble("timestamp");
    gps1.unix_sec = gps_node.getDouble("unix_sec");
    gps1.lat_deg = gps_node.getDouble("latitude_deg");
    gps1.lon_deg = gps_node.getDouble("longitude_deg");
    gps1.alt_m = gps_node.getDouble("altitude_m");
    gps1.vn_mps = gps_node.getDouble("vn_mps");
    gps1.ve_mps = gps_node.getDouble("ve_mps");
    gps1.vd_mps = gps_node.getDouble("vd_mps");

    string selected = config_nav_node.getString("select");
    if ( not ekf_inited and gps_node.getBool("settle") ) {
        if ( selected == "nav15" ) {
            ekf.init(imu1, gps1);
        } else if ( selected == "nav15_mag" ) {
            ekf_mag.init(imu1, gps1);
        }
        ekf_inited = true;
        printf("EKF: initialized\n");
    } else if ( ekf_inited ) {
        if ( selected == "nav15" ) {
            ekf.time_update(imu1);
        } else if ( selected == "nav15_mag" ) {
            ekf_mag.time_update(imu1);
        }
        if ( gps_node.getUInt("millis") > gps_last_millis ) {
            gps_last_millis = gps_node.getUInt("millis");
            if ( selected == "nav15" ) {
                ekf.measurement_update(gps1);
            } else if ( selected == "nav15_mag" ) {
                ekf_mag.measurement_update(imu1, gps1);
            }
            status = 2;         // ok
        }
        if ( selected == "nav15" ) {
            data = ekf.get_nav();
        } else if ( selected == "nav15_mag" ) {
            data = ekf_mag.get_nav();
        }

        // sanity checks in case degenerate input leads to the filter
        // blowing up.  look for nans (or even negative #'s) in the
        // covariance matrix.
        if ( std::isnan(data.Pp0) or std::isnan(data.Pv0) or std::isnan(data.Pa0)
             or (data.Pp0 < -0.1) or (data.Pv0 < -0.1) or (data.Pa0 < -0.1)
             or (data.Pp0 > 999999999.0) or (data.Pv0 > 999999999.0) or (data.Pa0 > 999999999.0) ) {
            printf("filter blew up...\n");
            status = 0;
            reinit();
        }
        if ( millis() - gps_last_millis >= 2000 ) {
            // last gps message > 2 seconds ago
            status = 1;         // no gps
        }

        // publish
        nav_node.setUInt("millis", imu_node.getUInt("millis"));
        nav_node.setDouble("latitude_deg", data.lat_rad * R2D);
        nav_node.setDouble("longitude_deg", data.lon_rad * R2D);
        nav_node.setInt("latitude_raw", intround(data.lat_rad * R2D * 10000000));
        nav_node.setInt("longitude_raw", intround(data.lon_rad * R2D * 10000000));
        nav_node.setDouble("altitude_m", data.alt_m);
        nav_node.setDouble("vn_mps", data.vn_mps);
        nav_node.setDouble("ve_mps", data.ve_mps);
        nav_node.setDouble("vd_mps", data.vd_mps);
        nav_node.setDouble("phi_rad", data.phi_rad);
        nav_node.setDouble("the_rad", data.the_rad);
        nav_node.setDouble("psi_rad", data.psi_rad);
        nav_node.setDouble("roll_deg", data.phi_rad * R2D);
        nav_node.setDouble("pitch_deg", data.the_rad * R2D);
        nav_node.setDouble("yaw_deg", data.psi_rad * R2D);
        nav_node.setDouble("p_bias", data.gbx);
        nav_node.setDouble("q_bias", data.gby);
        nav_node.setDouble("r_bias", data.gbz);
        nav_node.setDouble("ax_bias", data.abx);
        nav_node.setDouble("ay_bias", data.aby);
        nav_node.setDouble("az_bias", data.abz);
        nav_node.setDouble("Pp0", data.Pp0);
        nav_node.setDouble("Pp1", data.Pp1);
        nav_node.setDouble("Pp2", data.Pp2);
        nav_node.setDouble("Pv0", data.Pv0);
        nav_node.setDouble("Pv1", data.Pv1);
        nav_node.setDouble("Pv2", data.Pv2);
        nav_node.setDouble("Pa0", data.Pa0);
        nav_node.setDouble("Pa1", data.Pa1);
        nav_node.setDouble("Pa2", data.Pa2);

        // compute ground speed and track
        float hdg = (M_PI * 0.5 - atan2(data.vn_mps, data.ve_mps)) * R2D;
        float vel_ms = sqrt(data.vn_mps*data.vn_mps + data.ve_mps*data.ve_mps);
        nav_node.setDouble("groundtrack_deg", hdg);
        nav_node.setDouble("groundspeed_mps", vel_ms);

    } else {
        status = 0;             // not initialized
    }
    nav_node.setInt("status", status);

    nav_prof.stop();
}

void nav_mgr_t::reinit() {
    ekf_inited = false;
}

// global shared instance
nav_mgr_t *nav_mgr = nullptr;
