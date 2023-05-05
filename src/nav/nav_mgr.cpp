#include <math.h>

#include "nav_mgr.h"
#include "nav_constants.h"  // R2D

void nav_mgr_t::init() {
    config_nav_node = PropertyNode("/config/nav");
    gps_node = PropertyNode("/sensors/gps");
    imu_node = PropertyNode("/sensors/imu");
    nav_node = PropertyNode("/filters/nav");

    string selected = config_nav_node.getString("select");
    // fix me ...
    if ( selected == ""  ) {
        selected = "none";
        config_nav_node.setString("select", selected);
    } else {
        selected = "nav15";         // force (fixme)
    }
    console->printf("EKF: selected: %s\n", selected.c_str());
    configure();
    console->printf("configured ekf:\n");
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
    IMUdata imu1;
    imu1.time = imu_node.getDouble("timestamp");
    imu1.p = imu_node.getDouble("p_rps");
    imu1.q = imu_node.getDouble("q_rps");
    imu1.r = imu_node.getDouble("r_rps");
    imu1.ax = imu_node.getDouble("ax_mps2");
    imu1.ay = imu_node.getDouble("ay_mps2");
    imu1.az = imu_node.getDouble("az_mps2");
    imu1.hx = imu_node.getDouble("hx");
    imu1.hy = imu_node.getDouble("hy");
    imu1.hz = imu_node.getDouble("hz");

    GPSdata gps1;
    gps1.time = gps_node.getDouble("timestamp");
    gps1.unix_sec = gps_node.getDouble("unix_sec");
    gps1.lat = gps_node.getDouble("latitude_deg");
    gps1.lon = gps_node.getDouble("longitude_deg");
    gps1.alt = gps_node.getDouble("altitude_m");
    gps1.vn = gps_node.getDouble("vn_mps");
    gps1.ve = gps_node.getDouble("ve_mps");
    gps1.vd = gps_node.getDouble("vd_mps");

    string selected = config_nav_node.getString("select");
    if ( !ekf_inited and gps_node.getBool("settle") ) {
        if ( selected == "nav15" ) {
            ekf.init(imu1, gps1);
        } else if ( selected == "nav15_mag" ) {
            ekf_mag.init(imu1, gps1);
        }
        ekf_inited = true;
        console->printf("EKF: initialized\n");
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
        if ( std::isnan(data.Pp0) or std::isnan(data.Pv0)
             or std::isnan(data.Pa0) or (data.Pp0 < -0.1)
             or (data.Pv0 < -0.1) or (data.Pa0 < -0.1) ) {
            console->printf("filter blew up...\n");
            status = 0;
            reinit();
        }
        if ( AP_HAL::millis() - gps_last_millis >= 2000 ) {
            // last gps message > 2 seconds ago
            status = 1;         // no gps
        }

        // publish
        nav_node.setUInt("millis", imu_node.getUInt("millis"));
        nav_node.setDouble("latitude_deg", data.lat * R2D);
        nav_node.setDouble("longitude_deg", data.lon * R2D);
        nav_node.setInt("latitude_raw", intround(data.lat * R2D * 10000000));
        nav_node.setInt("longitude_raw", intround(data.lon * R2D * 10000000));
        nav_node.setDouble("altitude_m", data.alt);
        nav_node.setDouble("vn_mps", data.vn);
        nav_node.setDouble("ve_mps", data.ve);
        nav_node.setDouble("vd_mps", data.vd);
        nav_node.setDouble("phi_rad", data.phi);
        nav_node.setDouble("the_rad", data.the);
        nav_node.setDouble("psi_rad", data.psi);
        nav_node.setDouble("roll_deg", data.phi * R2D);
        nav_node.setDouble("pitch_deg", data.the * R2D);
        nav_node.setDouble("yaw_deg", data.psi * R2D);
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
    } else {
        status = 0;             // not initialized
    }
    nav_node.setInt("status", status);
}

void nav_mgr_t::reinit() {
    ekf_inited = false;
}

// global shared instance
nav_mgr_t nav_mgr;
