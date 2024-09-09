#include <Arduino.h>

#include "../../setup_board.h"
#include "../nodes.h"

#include "imu_mgr.h"

#include "MPU9250/MPU9250.h"

// IMU full scale ranges, DLPF bandwidth, interrupt SRD, and interrupt pin
const uint8_t MPU9250_SRD = 9;  // Data Output Rate = 1000 / (1 + SRD)

MPU9250 IMU;

// Setup imu common defaults:
void imu_mgr_t::defaults_common() {
    strapdown = Eigen::Matrix3f::Identity();
    for ( int i = 0; i < 9; i++ ) {
        imu_calib_node.setDouble("strapdown", strapdown.data()[i], i);
    }

    accel_affine = Eigen::Matrix4f::Identity();
    for ( int i = 0; i < 16; i++ ) {
        imu_calib_node.setDouble("accel_affine", accel_affine.data()[i], i);
    }

    mag_affine = Eigen::Matrix4f::Identity();
    for ( int i = 0; i < 16; i++ ) {
        imu_calib_node.setDouble("mag_affine", mag_affine.data()[i], i);
    }
}

// Update the R matrix (called after loading/receiving any new config message)
void imu_mgr_t::set_strapdown_calibration() {
    strapdown = Eigen::Matrix3f::Identity();
    if ( imu_calib_node.hasChild("strapdown") ) {
        for ( int i = 0; i < 3; i++ ) {
            for ( int j = 0; j < 3; j++ ) {
                strapdown(i,j) = imu_calib_node.getDouble("strapdown", i*3+j);
            }
        }
    }

    Serial.println("IMU strapdown calibration matrix:");
    for ( int i = 0; i < 3; i++ ) {
        Serial.print("  ");
        for ( int j = 0; j < 3; j++ ) {
            if ( strapdown(i,j) >= 0 ) {
                Serial.print(" ");
            }
            Serial.print(strapdown(i,j), 3);  Serial.print(" ");
        }
        Serial.println();
    }
}

// update the mag calibration matrix from the config structur
void imu_mgr_t::set_accel_calibration() {
    accel_affine = Eigen::Matrix4f::Identity();
    if ( imu_calib_node.hasChild("accel_affine") ) {
        for ( int i = 0; i < 4; i++ ) {
            for ( int j = 0; j < 4; j++ ) {
                accel_affine(i,j) = imu_calib_node.getDouble("accel_affine", i*4+j);
            }
        }
    }

    Serial.println("Accelerometer affine matrix:");
    for ( int i = 0; i < 4; i++ ) {
        Serial.print("  ");
        for ( int j = 0; j < 4; j++ ) {
            if ( accel_affine(i,j) >= 0 ) {
                Serial.print(" ");
            }
            Serial.print(accel_affine(i,j), 3); Serial.print(" ");
        }
        Serial.println();
    }
}

// update the mag calibration matrix from the config structur
void imu_mgr_t::set_mag_calibration() {
    mag_affine = Eigen::Matrix4f::Identity();
    if ( imu_calib_node.hasChild("mag_affine") ) {
        for ( int i = 0; i < 4; i++ ) {
            for ( int j = 0; j < 4; j++ ) {
                mag_affine(i,j) = imu_calib_node.getDouble("mag_affine", i*4+j);
            }
        }
    }

    Serial.println("Magnetometer affine matrix:");
    for ( int i = 0; i < 4; i++ ) {
        Serial.print("  ");
        for ( int j = 0; j < 4; j++ ) {
            if ( mag_affine(i,j) >= 0 ) {
                Serial.print(" ");
            }
            Serial.print(mag_affine(i,j), 3); Serial.print(" ");
        }
        Serial.println();
    }
}

// configure the IMU settings and setup the ISR to aquire the data
void imu_mgr_t::init() {
#if defined(MARMOT_V1)
    // Goldy3 has an mpu9250 on SPI CS line 24
    int pin = 24;
    printf("MPU9250 @ SPI pin: %d\n", pin);
    IMU.configure(pin);
#elif defined(AURA_V2) || defined(NORTHSTAR_V3)
    // Aura3 has an mpu9250 on I2C Addr 0x68
    int address = 0x68;
    printf("MPU9250 @ I2C Addr: 0x%02X\n", address);
    IMU.configure(address, &Wire);
#else
    printf("Error: problem with board configuration and MPU9250 (IMU) configuration.\n");
    return;
#endif

    // initialize the IMU, specify accelerometer and gyro ranges
    int beginStatus = IMU.begin(ACCEL_RANGE_4G, GYRO_RANGE_500DPS);
    if ( beginStatus < 0 ) {
        printf("\nIMU initialization unsuccessful.\n");
        printf("Check IMU wiring or try cycling power.\n");
        printf("\n");
        return;
    }

    // set the DLPF and interrupts
    int setFiltStatus = IMU.setFilt(DLPF_BANDWIDTH_41HZ, MPU9250_SRD);
    if ( setFiltStatus < 0 ) {
        printf("Filter initialization unsuccessful.\n");
        return;
    }

    hardware_inited = true;
    printf("MPU-9250 ready.\n");
}

// query the imu and update the structures
void imu_mgr_t::update() {
    if ( !hardware_inited ) {
        return;
    }

    string request = imu_node.getString("request");
    if ( request == "calibrate-accels" ) {
        imu_node.setString("request", "received: calibrate-accels");
        calib_accels.init();
    }

    imu_millis = millis();
    float ax_raw = 0.0, ay_raw = 0.0, az_raw = 0.0;
    float gx_raw = 0.0, gy_raw = 0.0, gz_raw = 0.0;
    float hx_raw = 0.0, hy_raw = 0.0, hz_raw = 0.0;
    IMU.getMotion10(&ax_raw, &ay_raw, &az_raw,
                    &gx_raw, &gy_raw, &gz_raw,
                    &hx_raw, &hy_raw, &hz_raw, &temp_C);
    accels_raw << ax_raw, ay_raw, az_raw, 1.0;
    gyros_raw << gx_raw, gy_raw, gz_raw;

    Vector3f mags_precal;
    mags_precal << hx_raw, hy_raw, hz_raw;
    mags_raw.head(3) = strapdown * mags_precal;
    mags_raw(3) = 1.0;

    accels_cal = accel_affine * accels_raw;
    gyros_cal = strapdown * gyros_raw;

    //accels_cal(0) = ax_cal.calibrate(accels_nocal(0), temp_C);
    //accels_cal(1) = ay_cal.calibrate(accels_nocal(1), temp_C);
    //accels_cal(2) = az_cal.calibrate(accels_nocal(2), temp_C);

    mags_cal = mag_affine * mags_raw;

    if ( gyros_calibrated < 2 ) {
        calibrate_gyros();
    } else {
        gyros_cal.segment(0,3) -= gyro_startup_bias;
    }

    // publish
    imu_node.setUInt("millis", imu_millis);
    imu_node.setDouble("timestamp", imu_millis / 1000.0);
    imu_node.setDouble("ax_raw", accels_raw(0));
    imu_node.setDouble("ay_raw", accels_raw(1));
    imu_node.setDouble("az_raw", accels_raw(2));
    imu_node.setDouble("hx_raw", mags_raw(0));
    imu_node.setDouble("hy_raw", mags_raw(1));
    imu_node.setDouble("hz_raw", mags_raw(2));
    imu_node.setDouble("ax_mps2", accels_cal(0));
    imu_node.setDouble("ay_mps2", accels_cal(1));
    imu_node.setDouble("az_mps2", accels_cal(2));
    imu_node.setDouble("p_rps", gyros_cal(0));
    imu_node.setDouble("q_rps", gyros_cal(1));
    imu_node.setDouble("r_rps", gyros_cal(2));
    imu_node.setDouble("hx", mags_cal(0));
    imu_node.setDouble("hy", mags_cal(1));
    imu_node.setDouble("hz", mags_cal(2));
    imu_node.setDouble("temp_C", temp_C);
    imu_node.setUInt("gyros_calibrated", gyros_calibrated);

    calib_accels.update();      // run if requested
}


// stay alive for up to 15 seconds looking for agreement between a 1
// second low pass filter and a 0.1 second low pass filter.  If these
// agree (close enough) for 4 consecutive seconds, then we calibrate
// with the 1 sec low pass filter value.  If time expires, the
// calibration fails and we run with raw gyro values.
void imu_mgr_t::calibrate_gyros() {
    if ( gyros_calibrated == 0 ) {
        Serial.print("Initialize gyro calibration: ");
        slow = gyros_cal.segment(0,3);
        fast = gyros_cal.segment(0,3);
        total_timer = 0;
        good_timer = 0;
        output_timer = 0;
        gyros_calibrated = 1;
    }

    fast = 0.95 * fast + 0.05 * gyros_cal.segment(0,3);
    slow = 0.995 * fast + 0.005 * gyros_cal.segment(0,3);
    // use 'slow' filter value for calibration while calibrating
    gyro_startup_bias << slow;

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
        gyro_startup_bias = slow;
        gyros_calibrated = 2;
        // update(); // update imu_calib values before anything else get's a chance to read them // FIXME???
        Serial.print("Average gyro startup bias: ");
        Serial.print(gyro_startup_bias(0), 3); Serial.print(" ");
        Serial.print(gyro_startup_bias(1), 3); Serial.print(" ");
        Serial.print(gyro_startup_bias(2), 3); Serial.println();
        if ( total_timer > 15000 ) {
            Serial.println("gyro init: too much motion, using best average guess.");
        } else {
            Serial.println("gyro init: success.");
        }
    }
}