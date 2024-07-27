// IMU wrapper class

#pragma once

#include <Arduino.h>

#include <eigen.h>
using namespace Eigen;

#include "../calibration/calib_accels.h"
#include "../util/cal_temp.h"

class imu_mgr_t {

private:

    bool hardware_inited = false;
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
    void init();
    void update();
};
