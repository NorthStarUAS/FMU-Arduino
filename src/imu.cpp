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
    R = Matrix3f::Identity();
    for ( int i = 0; i < 9; i++ ) {
        // no need to worry about row vs. column major here (symmetrical ident)
        config.orientation[i] = R.data()[i];
    }
}

// Setup imu defaults:
// Aura3 has mpu9250 on I2C Addr 0x68
void imu_t::defaults_aura3() {
    config.interface = 1;       // i2c
    config.pin_or_address = 0x68; // mpu9250 i2c addr
    R = Matrix3f::Identity();
    for ( int i = 0; i < 9; i++ ) {
        // no need to worry about row vs. column major here (symmetrical ident)
        config.orientation[i] = R.data()[i];
    }
}

// Update the R matrix (called after loading/receiving any new config message)
void imu_t::set_orientation() {
    // config.orientation is row major, but internally Eigen defaults
    // to column major.
    R = Matrix<float, 3, 3, RowMajor>(config.orientation);
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
    for ( int i = 0; i < 3; i++ ) {
        for ( int j = 0; j < 3; j++ ) {
            Serial.print(R(i,j), 2);
            Serial.print(" ");
        }
        Serial.println();
    }
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
    
    accels_raw << ax_raw, ay_raw, az_raw;
    gyros_raw << gx_raw, gy_raw, gz_raw;
    mags_raw << hx_raw, hy_raw, hz_raw;
    
    accels = R * accels_raw;
    gyros = R * gyros_raw;
    mags = R * mags_raw;
    
    if ( gyros_calibrated < 2 ) {
        calibrate_gyros();
    } else {
        gyros -= gyro_calib;
    }
}


// stay alive for up to 15 seconds looking for agreement between a 1
// second low pass filter and a 0.1 second low pass filter.  If these
// agree (close enough) for 4 consecutive seconds, then we calibrate
// with the 1 sec low pass filter value.  If time expires, the
// calibration fails and we run with raw gyro values.
void imu_t::calibrate_gyros() {
    if ( gyros_calibrated == 0 ) {
        Serial.print("Initialize gyro calibration: ");
        slow = gyros_raw;
        fast = gyros_raw;
        total_timer = 0;
        good_timer = 0;
        output_timer = 0;
        gyros_calibrated = 1;
    }

    fast = 0.95 * fast + 0.05 * gyros_raw;
    slow = 0.995 * fast + 0.005 * gyros_raw;
    // use 'slow' filter value for calibration while calibrating
    gyro_calib << slow;

    float max = (slow - fast).cwiseAbs().maxCoeff();
    if ( max > cutoff ) {
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
        gyro_calib = slow;
        gyros_calibrated = 2;
        // update(); // update imu_calib values before anything else get's a chance to read them // FIXME???
        Serial.print("Average gyros: ");
        Serial.print(gyro_calib(0), 4);
        Serial.print(" ");
        Serial.print(gyro_calib(1), 4);
        Serial.print(" ");
        Serial.print(gyro_calib(2), 4);
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
