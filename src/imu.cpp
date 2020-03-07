// #include <Eigen.h>
// #include <Eigen/Core>
// using namespace Eigen;

#include "imu.h"

#include "sensors/MPU9250/MPU9250.h"

// IMU full scale ranges, DLPF bandwidth, interrupt SRD, and interrupt pin
const uint8_t MPU9250_SRD = 9;  // Data Output Rate = 1000 / (1 + SRD)

MPU9250 IMU;

// Setup imu defaults:
// Goldy3 has mpu9250 on SPI CS line 24
void imu_t::defaults_goldy3() {
    config.interface = 0;       // SPI
    config.pin_or_address = 24; // CS pin
    float ident[] = { 1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0};
    for ( int i = 0; i < 9; i++ ) {
        config.orientation[i] = ident[i];
    }
}

// Setup imu defaults:
// Aura3 has mpu9250 on I2C Addr 0x68
void imu_t::defaults_aura3() {
    config.interface = 1;       // i2c
    config.pin_or_address = 0x68; // mpu9250 i2c addr
    float ident[] = { 1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0};
    for ( int i = 0; i < 9; i++ ) {
        config.orientation[i] = ident[i];
    }
}

// configure the IMU settings and setup the ISR to aquire the data
void imu_t::setup() {
    if ( config.interface == 0 ) {
        // SPI
        Serial.print("MPU9250 @ SPI pin: ");
        Serial.println(config.pin_or_address);
        IMU.configure(config.pin_or_address);
    } else if ( config.interface == 1 ) {
        Serial.print("MPU9250 @ I2C Addr: 0x");
        Serial.println(config.pin_or_address, HEX);
        IMU.configure(config.pin_or_address, &Wire);
    } else {
        Serial.println("Error: problem with MPU9250 (IMU) configuration");
    }
    
    // initialize the IMU, specify accelerometer and gyro ranges
    int beginStatus = IMU.begin(ACCEL_RANGE_4G, GYRO_RANGE_500DPS);
    if ( beginStatus < 0 ) {
        Serial.println("\nIMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.println();
        delay(1000);
        return;
    }

    // set the DLPF and interrupts
    int setFiltStatus = IMU.setFilt(DLPF_BANDWIDTH_41HZ, MPU9250_SRD);
    if ( setFiltStatus < 0 ) {
        Serial.println("Filter initialization unsuccessful");
        delay(1000);
        return;
    }

    Serial.println("MPU-9250 ready.");
    for ( int i = 0; i < 9; i++ ) {
        Serial.print(config.orientation[i], 2);
        Serial.print(" ");
        if ( i == 2 or i == 5 or i == 8 ) {
            Serial.println();
        }
    }
}

void imu_t::rotate(float v0, float v1, float v2,
                   float *r0, float *r1, float *r2)
{
    *r0 = v0*config.orientation[0] + v1*config.orientation[1] + v2*config.orientation[2];
    *r1 = v0*config.orientation[3] + v1*config.orientation[4] + v2*config.orientation[5];
    *r2 = v0*config.orientation[6] + v1*config.orientation[7] + v2*config.orientation[8];
}

// query the imu and update the structures
void imu_t::update() {
    imu_micros = micros();
    float ax_raw, ay_raw, az_raw;
    float gx_raw, gy_raw, gz_raw;
    float hx_raw, hy_raw, hz_raw;
    float t;
    IMU.getMotion10(&ax_raw, &ay_raw, &az_raw,
                    &gx_raw, &gy_raw, &gz_raw,
                    &hx_raw, &hy_raw, &hz_raw, &t);
    
    // rotate into aircraft body frame
    rotate(ax_raw, ay_raw, az_raw, &ax, &ay, &az);
    rotate(gx_raw, gy_raw, gz_raw, &p, &q, &r);
    rotate(hx_raw, hy_raw, hz_raw, &hx, &hy, &hz);
    
    if ( gyros_calibrated < 2 ) {
        calibrate_gyros(p, q, r);
    } else {
        p -= p_calib;
        q -= q_calib;
        r -= r_calib;
    }
}


// stay alive for up to 15 seconds looking for agreement between a 1
// second low pass filter and a 0.1 second low pass filter.  If these
// agree (close enough) for 4 consecutive seconds, then we calibrate
// with the 1 sec low pass filter value.  If time expires, the
// calibration fails and we run with raw gyro values.
void imu_t::calibrate_gyros(float gx, float gy, float gz) {
    static const float cutoff = 0.005;
    static float gxs = 0.0;
    static float gys = 0.0;
    static float gzs = 0.0;
    static float gxf = 0.0;
    static float gyf = 0.0;
    static float gzf = 0.0;
    static elapsedMillis total_timer = 0;
    static elapsedMillis good_timer = 0;
    static elapsedMillis output_timer = 0;

    if ( gyros_calibrated == 0 ) {
        Serial.print("Initialize gyro calibration: ");
        gxs = gx;
        gys = gy;
        gzs = gz;
        gxf = gx;
        gyf = gy;
        gzf = gz;
        total_timer = 0;
        good_timer = 0;
        output_timer = 0;
        gyros_calibrated = 1;
    }
    
    gxf = 0.95 * gxf + 0.05 * gx;
    gyf = 0.95 * gyf + 0.05 * gy;
    gzf = 0.95 * gzf + 0.05 * gz;
    gxs = 0.995 * gxs + 0.005 * gx;
    gys = 0.995 * gys + 0.005 * gy;
    gzs = 0.995 * gzs + 0.005 * gz;
    
    // use 'slow' filter value for calibration while calibrating
    p_calib = gxs;
    q_calib = gys;
    r_calib = gzs;

    float dx = fabs(gxs - gxf);
    float dy = fabs(gys - gyf);
    float dz = fabs(gzs - gzf);
    if ( dx > cutoff || dy > cutoff || dz > cutoff ) {
        good_timer = 0;
    }
    if ( output_timer >= 1000 ) {
        output_timer = 0;
        if ( good_timer < 1000 ) {
            Serial.print("x");
        } else {
            Serial.print("*");
        }
    }
    if ( good_timer > 4100 || total_timer > 15000 ) {
        Serial.println();
        // set gyro zero points from the 'slow' filter.
        p_calib = gxs;
        q_calib = gys;
        r_calib = gzs;
        gyros_calibrated = 2;
        // update(); // update imu_calib values before anything else get's a chance to read them // FIXME???
        Serial.print("Average gyros: ");
        Serial.print(p_calib, 4);
        Serial.print(" ");
        Serial.print(q_calib, 4);
        Serial.print(" ");
        Serial.print(r_calib, 4);
        Serial.println();
        if ( total_timer > 15000 ) {
            Serial.println("gyro init: too much motion, using best average guess.");
        } else {
            Serial.println("gyro init: success.");
        }
    }
}

// global shared instance
imu_t imu;
