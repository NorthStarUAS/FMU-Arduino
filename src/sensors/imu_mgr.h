// IMU wrapper class

#pragma once

#include <Arduino.h>

#include <eigen.h>
using namespace Eigen;

#include "../props2.h"
#include "../calibration/calib_accels.h"
#include "../util/cal_temp.h"

class imu_mgr_t {

private:

    Matrix3f strapdown = Matrix3f::Identity();
    Matrix4f accel_affine = Matrix4f::Identity();
    Matrix4f mag_affine = Matrix4f::Identity();
    // gyro zero'ing stuff
    const float cutoff = 0.005;
    elapsedMillis total_timer = 0;
    elapsedMillis good_timer = 0;
    elapsedMillis output_timer = 0;
    Vector3f slow = Vector3f::Zero();
    Vector3f fast = Vector3f::Zero();
    Vector3f gyro_startup_bias = Vector3f::Zero();
    void calibrate_gyros();
    CalTemp ax_cal;
    CalTemp ay_cal;
    CalTemp az_cal;

    calib_accels_t calib_accels;

    PropertyNode imu_node;
    PropertyNode imu_calib_node;
    PropertyNode sim_node;

public:
    // 0 = uncalibrated, 1 = calibration in progress, 2 = calibration finished
    int gyros_calibrated = 0;
    unsigned long imu_millis = 0;
    // raw/uncorrected sensor values
    Vector4f accels_raw = Vector4f::Zero();
    Vector3f gyros_raw = Vector3f::Zero();
    Vector4f mags_raw = Vector4f::Zero();
    // rotation corrected sensor values
    //Vector3f accels_nocal = Vector3f::Zero();
    //Vector3f gyros_nocal = Vector3f::Zero();
    //Vector3f mags_nocal = Vector3f::Zero();
    Vector4f accels_cal = Vector4f::Zero();
    Vector3f gyros_cal = Vector3f::Zero();
    Vector4f mags_cal = Vector4f::Zero();
    float temp_C = 0.0;

    // void defaults_goldy3();
    // void defaults_aura3();
    void defaults_common();
    void set_strapdown_calibration();
    void set_accel_calibration();
    void set_mag_calibration();
    void setup();
    void update();

    // notational convenience/clarity
    // inline float get_ax_raw() { return accels_raw(0); }
    // inline float get_ay_raw() { return accels_raw(1); }
    // inline float get_az_raw() { return accels_raw(2); }
    // inline float get_p_raw() { return gyros_raw(0); }
    // inline float get_q_raw() { return gyros_raw(1); }
    // inline float get_r_raw() { return gyros_raw(2); }
    // inline float get_hx_raw() { return mags_raw(0); }
    // inline float get_hy_raw() { return mags_raw(1); }
    // inline float get_hz_raw() { return mags_raw(2); }
    // inline float get_ax_cal() { return accels_cal(0); }
    // inline float get_ay_cal() { return accels_cal(1); }
    // inline float get_az_cal() { return accels_cal(2); }
    // inline float get_p_cal() { return gyros_cal(0); }
    // inline float get_q_cal() { return gyros_cal(1); }
    // inline float get_r_cal() { return gyros_cal(2); }
    // inline float get_hx_cal() { return mags_cal(0); }
    // inline float get_hy_cal() { return mags_cal(1); }
    // inline float get_hz_cal() { return mags_cal(2); }
    // inline float get_tempC() { return tempC; }
};

extern imu_mgr_t imu_mgr;
