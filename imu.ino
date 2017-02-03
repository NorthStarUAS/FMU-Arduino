#include <MPU9250.h>

/* MPU-9250 IMU */
MPU9250 IMU(0x68, 0);

// IMU full scale ranges, DLPF bandwidth, interrupt SRD, and interrupt pin
const mpu9250_accel_range MPU_9250_ACCEL_RANGE = ACCEL_RANGE_8G;
const mpu9250_gyro_range MPU_9250_GYRO_RANGE = GYRO_RANGE_250DPS;
const mpu9250_dlpf_bandwidth MPU9250_BANDWIDTH = DLPF_BANDWIDTH_41HZ;
const uint8_t MPU9250_SRD = 9;  // Data Output Rate = 1000 / (1 + SRD)
const uint8_t SYNC_PIN = 2;

// any code that reads imu_sensors_shared should protect those reads with cli() / sei() calls because
// this array is modified by an ISR that could be called at any time, even in the middle of a variable
// read which could lead to corrupted values.

volatile float imu_sensors_shared[10]; // the volatile (shared) storage for the imu sensors
float imu_raw[10]; // the 'safe' but raw version of the imu sensors
float gyro_calib[3] = { 0.0, 0.0, 0.0 };


// configure the IMU settings and setup the ISR to aquire the data
void imu_setup() {
    // initialize the IMU
    if(IMU.begin(MPU_9250_ACCEL_RANGE,MPU_9250_GYRO_RANGE) < 0){}
    
    // set the DLPF and interrupts
    //if(IMU.setFilt(MPU9250_BANDWIDTH,MPU9250_SRD) < 0){}
    //pinMode(SYNC_PIN,INPUT);
    //attachInterrupt(SYNC_PIN, dataAcquisition, RISING);
}


// ISR, system automatically disables interrupts when running
void dataAcquisition() {
    // imu data
    float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
    IMU.getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
    imu_sensors_shared[0] = ax;
    imu_sensors_shared[1] = ay;
    imu_sensors_shared[2] = az;
    imu_sensors_shared[3] = gx;
    imu_sensors_shared[4] = gy;
    imu_sensors_shared[5] = gz;
    imu_sensors_shared[6] = hx;
    imu_sensors_shared[7] = hy;
    imu_sensors_shared[8] = hz;
    imu_sensors_shared[9] = t;
    new_imu_data = true;
    imu_counter++;
}


// copy the dangerous 'volatile' shared version of imu data to the safe global copy.
void update_imu() {
    for ( int i = 0; i < 10; i++ ) {
        imu_raw[i] = imu_sensors_shared[i];
    }
    for ( int i = 0; i < 10; i++ ) {
        imu_calib[i] = imu_raw[i];
    }
    if ( gyros_calibrated < 2 ) {
        calibrate_gyros();
    } else {
        for ( int i = 0; i < 3; i++ ) {
            imu_calib[i+3] -= gyro_calib[i];
        }
    }
}


void imu_print() {
    for ( int i = 0; i < 6; i++ ) {
        Serial.print(imu_calib[i],3); Serial.print(" ");
    }
    Serial.println(imu_calib[9],1);
}


// stay alive for up to 15 seconds looking for agreement between a 1 second low pass filter and a 0.1 second
// low pass filter.  If these agree (close enough) for 4 consecutive seconds, then we calibrate with the
// 1 sec low pass filter value.  If time expires the calibration fails and we run with raw gyro values.
void calibrate_gyros() {
    static float gxs = imu_raw[3];
    static float gys = imu_raw[4];
    static float gzs = imu_raw[5];
    static float gxf = imu_raw[3];
    static float gyf = imu_raw[4];
    static float gzf = imu_raw[5];
    static const float cutoff = 0.005;
    static elapsedMillis total_timer = 0;
    static elapsedMillis good_timer = 0;
    static elapsedMillis output_timer = 0;

    // zero the calibration
    gyro_calib[0] = gxs;
    gyro_calib[1] = gys;
    gyro_calib[2] = gzs;

    if ( gyros_calibrated == 0 ) {
        Serial.print("Calibrating gyros: ");
        gyros_calibrated = 1;
    }
    
    gxf = 0.9 * gxf + 0.1 * imu_raw[3];
    gyf = 0.9 * gyf + 0.1 * imu_raw[4];
    gzf = 0.9 * gzf + 0.1 * imu_raw[5];
    gxs = 0.99 * gxs + 0.01 * imu_raw[3];
    gys = 0.99 * gys + 0.01 * imu_raw[4];
    gzs = 0.99 * gzs + 0.01 * imu_raw[5];
    
    float dx = fabs(gxs - gxf);
    float dy = fabs(gys - gyf);
    float dz = fabs(gzs - gzf);
    if ( dx > cutoff || dy > cutoff || dz > cutoff ) {
        good_timer = 0;
    }
    if ( output_timer > 1000 ) {
        output_timer = 0;
        if ( good_timer < 1000 ) {
            Serial.print("X");
        } else {
            Serial.print("*");
        }
    }
    if (good_timer > 4000) {
        // set gyro zero points from the 'slow' filter.
        gyro_calib[0] = gxs;
        gyro_calib[1] = gys;
        gyro_calib[2] = gzs;
        gyros_calibrated = 2;
        update_imu(); // update imu_calib values before anything else get's a chance to read them
        Serial.println(" :)");
        Serial.print("Average gyros: ");
        Serial.print(gyro_calib[0],4);
        Serial.print(" ");
        Serial.print(gyro_calib[1],4);
        Serial.print(" ");
        Serial.print(gyro_calib[2],4);
        Serial.println();
    } else if (total_timer > 15000) {
        gyros_calibrated = 2;
        Serial.println(" :(");
    }
}

