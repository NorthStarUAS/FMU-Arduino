// IMU wrapper class

#pragma once

#include <Arduino.h>

#include <Eigen.h>
#include <Eigen/Core>
using namespace Eigen;

class imu_t {
private:
    Matrix3f R = Matrix3f::Identity();
    Vector3f accels_raw = Vector3f::Zero();
    Vector3f gyros_raw = Vector3f::Zero();
    Vector3f mags_raw = Vector3f::Zero();
    // gyro zero'ing stuff
    const float cutoff = 0.005;
    elapsedMillis total_timer = 0;
    elapsedMillis good_timer = 0;
    elapsedMillis output_timer = 0;
    Vector3f slow = Vector3f::Zero();
    Vector3f fast = Vector3f::Zero();
    Vector3f gyro_calib = Vector3f::Zero();
    void calibrate_gyros();

public:
    // 0 = uncalibrated, 1 = calibration in progress, 2 = calibration finished
    int gyros_calibrated = 0;
    unsigned long imu_micros = 0;
    // rotation corrected sensor values
    Vector3f accels = Vector3f::Zero();
    Vector3f gyros = Vector3f::Zero();
    Vector3f mags = Vector3f::Zero();
    float temp = 0.0;
    
    void defaults_goldy3();
    void defaults_aura3();
    void set_orientation();
    void setup();
    void update();
    // notational convenience/clarity
    inline float get_ax() { return accels(0); }
    inline float get_ay() { return accels(1); }
    inline float get_az() { return accels(2); }
    inline float get_p() { return gyros(0); }
    inline float get_q() { return gyros(1); }
    inline float get_r() { return gyros(2); }
    inline float get_hx() { return mags(0); }
    inline float get_hy() { return mags(1); }
    inline float get_hz() { return mags(2); }
    inline float get_temp() { return temp; }
};

extern imu_t imu;
