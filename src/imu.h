// IMU wrapper class

#pragma once

#include <Arduino.h>

#include <Eigen.h>
#include <Eigen/Core>
using namespace Eigen;

#include "util/cal_temp.h"

class imu_t {
private:
    Matrix3f R = Matrix3f::Identity();
    Matrix4f mag_affine = Matrix4f::Identity();
    //Vector3f accels_raw = Vector3f::Zero();
    //Vector3f gyros_raw = Vector3f::Zero();
    //Vector3f mags_raw = Vector3f::Zero();
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

public:
    // 0 = uncalibrated, 1 = calibration in progress, 2 = calibration finished
    int gyros_calibrated = 0;
    unsigned long imu_millis = 0;
    // rotation corrected sensor values
    Vector3f accels_nocal = Vector3f::Zero();
    Vector3f gyros_nocal = Vector3f::Zero();
    Vector3f mags_nocal = Vector3f::Zero();
    Vector3f accels_cal = Vector3f::Zero();
    Vector3f gyros_cal = Vector3f::Zero();
    Vector4f mags_cal = Vector4f::Zero();
    float tempC = 0.0;
    
    void defaults_goldy3();
    void defaults_aura3();
    void defaults_common();
    void set_orientation();
    void set_accel_calibration();
    void set_mag_calibration();
    void setup();
    void update();
    // notational convenience/clarity
    inline float get_ax_nocal() { return accels_nocal(0); }
    inline float get_ay_nocal() { return accels_nocal(1); }
    inline float get_az_nocal() { return accels_nocal(2); }
    inline float get_hx_nocal() { return mags_nocal(0); }
    inline float get_hy_nocal() { return mags_nocal(1); }
    inline float get_hz_nocal() { return mags_nocal(2); }
    inline float get_ax_cal() { return accels_cal(0); }
    inline float get_ay_cal() { return accels_cal(1); }
    inline float get_az_cal() { return accels_cal(2); }
    inline float get_p_cal() { return gyros_cal(0); }
    inline float get_q_cal() { return gyros_cal(1); }
    inline float get_r_cal() { return gyros_cal(2); }
    inline float get_hx_cal() { return mags_cal(0); }
    inline float get_hy_cal() { return mags_cal(1); }
    inline float get_hz_cal() { return mags_cal(2); }
    inline float get_tempC() { return tempC; }
};

extern imu_t imu;
