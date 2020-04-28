#include "config.h"

#include "imu.h"

#include "sensors/MPU9250/MPU9250.h"

// IMU full scale ranges, DLPF bandwidth, interrupt SRD, and interrupt pin
const uint8_t MPU9250_SRD = 9;  // Data Output Rate = 1000 / (1 + SRD)

MPU9250 IMU;

// Setup imu defaults:
// Goldy3 has mpu9250 on SPI CS line 24
void imu_t::defaults_goldy3() {
    config.imu.interface = 0;       // SPI
    config.imu.pin_or_address = 24; // CS pin
    defaults_common();
}

// Setup imu defaults:
// Aura3 has mpu9250 on I2C Addr 0x68
void imu_t::defaults_aura3() {
    config.imu.interface = 1;       // i2c
    config.imu.pin_or_address = 0x68; // mpu9250 i2c addr
    defaults_common();
}

// Setup imu defaults:
// Aura3 has mpu9250 on I2C Addr 0x68
void imu_t::defaults_common() {
    Matrix3f strapdown3x3 = Matrix3f::Identity();
    for ( int i = 0; i < 9; i++ ) {
        // no need to worry about row vs. column major here (symmetrical ident)
        config.imu.strapdown_calib[i] = strapdown3x3.data()[i];
    }
    strapdown = Matrix4f::Identity();

    for ( int i = 0; i < 3; i++ ) {
        config.imu.accel_scale[i] = 1.0;
    }
    for ( int i = 0; i < 3; i++ ) {
        config.imu.accel_translate[i] = 0.0;
    }
    accel_affine = Matrix4f::Identity();
    
    //config.imu.min_temp = 27.0;
    //config.imu.max_temp = 27.0;
    // for ( int i = 0; i < 3; i++ ) {
    //    config.imu.ax_coeff[i] = 0.0;
    //    config.imu.ay_coeff[i] = 0.0;
    //    config.imu.az_coeff[i] = 0.0;
    //}
    mag_affine = Matrix4f::Identity();
    for ( int i = 0; i < 16; i++ ) {
        // no need to worry about row vs. column major here (symmetrical ident)
        config.imu.mag_affine[i] = mag_affine.data()[i];
    }
}

// Update the R matrix (called after loading/receiving any new config message)
void imu_t::set_strapdown_calibration() {
    // config.imu.orientation is row major, but internally Eigen defaults
    // to column major.
    Matrix3f strapdown3x3 = Matrix<float, 3, 3, RowMajor>(config.imu.strapdown_calib);
    strapdown = Matrix4f::Identity();
    strapdown.block(0,0,3,3) = strapdown3x3;
    Matrix4f scale = Matrix4f::Identity();
    for (int i = 0; i < 3; i++ ) {
        scale(i,i) = config.imu.accel_scale[i];
    }
    Matrix4f translate = Matrix4f::Identity();
    for (int i = 0; i < 3; i++ ) {
        // column major
        translate(i,3) = config.imu.accel_translate[i];
    }
    accel_affine = translate * strapdown * scale;
    Serial.println("Accel affine calibration matrix:");
    for ( int i = 0; i < 4; i++ ) {
        Serial.print("  ");
        for ( int j = 0; j < 4; j++ ) {
            Serial.print(accel_affine(i,j), 4);
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println("IMU strapdown calibration matrix:");
    for ( int i = 0; i < 4; i++ ) {
        Serial.print("  ");
        for ( int j = 0; j < 4; j++ ) {
            Serial.print(strapdown(i,j), 4);
            Serial.print(" ");
        }
        Serial.println();
    }
}

// setup accel temp calibration
//void imu_t::set_accel_calibration() {
    //ax_cal.init(config.imu.ax_coeff, config.imu.min_temp, config.imu.max_temp);
    //ay_cal.init(config.imu.ay_coeff, config.imu.min_temp, config.imu.max_temp);
    //az_cal.init(config.imu.az_coeff, config.imu.min_temp, config.imu.max_temp);
//}

// update the mag calibration matrix from the config structur
void imu_t::set_mag_calibration() {
    mag_affine = Matrix<float, 4, 4, RowMajor>(config.imu.mag_affine);
}

// configure the IMU settings and setup the ISR to aquire the data
void imu_t::setup() {
    if ( config.imu.interface == 0 ) {
        // SPI
        Serial.print("MPU9250 @ SPI pin: ");
        Serial.println(config.imu.pin_or_address);
        IMU.configure(config.imu.pin_or_address);
    } else if ( config.imu.interface == 1 ) {
        Serial.print("MPU9250 @ I2C Addr: 0x");
        Serial.println(config.imu.pin_or_address, HEX);
        IMU.configure(config.imu.pin_or_address, &Wire);
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

    // Serial.print("Accel calibration temp range: ");
    // Serial.print(config.imu.min_temp, 1);
    // Serial.print(" - ");
    // Serial.println(config.imu.max_temp, 1);
    // Serial.print("ax_coeff: ");
    // Serial.print(config.imu.ax_coeff[0], 4); Serial.print(" ");
    // Serial.print(config.imu.ax_coeff[1], 4); Serial.print(" ");
    // Serial.println(config.imu.ax_coeff[2], 4);
    // Serial.print("ay_coeff: ");
    // Serial.print(config.imu.ay_coeff[0], 4); Serial.print(" ");
    // Serial.print(config.imu.ay_coeff[1], 4); Serial.print(" ");
    // Serial.println(config.imu.ay_coeff[2], 4);
    // Serial.print("az_coeff: ");
    // Serial.print(config.imu.az_coeff[0], 4); Serial.print(" ");
    // Serial.print(config.imu.az_coeff[1], 4); Serial.print(" ");
    // Serial.println(config.imu.az_coeff[2], 4);
    Serial.println("Magnetometer calibration matrix:");
    for ( int i = 0; i < 4; i++ ) {
        Serial.print("  ");
        for ( int j = 0; j < 4; j++ ) {
            Serial.print(mag_affine(i,j), 4);
            Serial.print(" ");
        }
        Serial.println();
    }
}

// query the imu and update the structures
void imu_t::update() {
    imu_millis = millis();
    float ax_raw, ay_raw, az_raw;
    float gx_raw, gy_raw, gz_raw;
    float hx_raw, hy_raw, hz_raw;
    IMU.getMotion10(&ax_raw, &ay_raw, &az_raw,
                    &gx_raw, &gy_raw, &gz_raw,
                    &hx_raw, &hy_raw, &hz_raw, &tempC);
    
    //Vector4f accels_raw;
    //Vector3f gyros_raw;
    //Vector4f mags_raw;
    accels_raw << ax_raw, ay_raw, az_raw, 1.0;
    gyros_raw << gx_raw, gy_raw, gz_raw, 1.0;
    mags_raw << hx_raw, hy_raw, hz_raw, 1.0;
    
    accels_cal = accel_affine * accels_raw;
    gyros_cal = strapdown * gyros_raw;
    /* FIXME!!!! */ //mags_cal = strapdown * mags_raw;

    //accels_cal(0) = ax_cal.calibrate(accels_nocal(0), tempC);
    //accels_cal(1) = ay_cal.calibrate(accels_nocal(1), tempC);
    //accels_cal(2) = az_cal.calibrate(accels_nocal(2), tempC);
        
    Vector4f hs;
    /* FIXME!!! */ hs << mags_raw(0), mags_raw(1), mags_raw(2), 1.0;
    mags_cal = mag_affine * hs;
    
    if ( gyros_calibrated < 2 ) {
        calibrate_gyros();
    } else {
        gyros_cal.segment(0,3) -= gyro_startup_bias;
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
        Serial.print(gyro_startup_bias(0), 4);
        Serial.print(" ");
        Serial.print(gyro_startup_bias(1), 4);
        Serial.print(" ");
        Serial.print(gyro_startup_bias(2), 4);
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
