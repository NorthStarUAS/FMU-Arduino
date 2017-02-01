#include <MPU9250.h>

/* MPU-9250 IMU */
MPU9250 IMU(0x68, 0);

// IMU full scale ranges, DLPF bandwidth, interrupt SRD, and interrupt pin
const mpu9250_accel_range MPU_9250_ACCEL_RANGE = ACCEL_RANGE_8G;
const mpu9250_gyro_range MPU_9250_GYRO_RANGE = GYRO_RANGE_250DPS;
const mpu9250_dlpf_bandwidth MPU9250_BANDWIDTH = DLPF_BANDWIDTH_41HZ;
const uint8_t MPU9250_SRD = 9;  // Data Output Rate = 1000 / (1 + SRD)
const uint8_t SYNC_PIN = 2;

volatile float imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz, imu_hx, imu_hy, imu_hz, imu_t;

void imu_setup() {
    // initialize the IMU
    if(IMU.begin(MPU_9250_ACCEL_RANGE,MPU_9250_GYRO_RANGE) < 0){}
    
    // set the DLPF and interrupts
    if(IMU.setFilt(MPU9250_BANDWIDTH,MPU9250_SRD) < 0){}
    pinMode(SYNC_PIN,INPUT);
    attachInterrupt(SYNC_PIN, dataAcquisition, RISING);
}


void dataAcquisition() {
    // imu data
    float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
    IMU.getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
    cli();
    imu_gx = gx;
    imu_gy = gy;
    imu_gz = gz;
    sei();
}


void calibrate_gyros() {
    float gxs = imu_gx;
    float gys = imu_gy;
    float gzs = imu_gz;
    float gxf = imu_gx;
    float gyf = imu_gy;
    float gzf = imu_gz;
    const float cutoff = 0.005;
    elapsedMillis total_timer = 0;
    elapsedMillis good_timer = 0;
    elapsedMillis output_timer = 0;

    Serial.print("Calibrating gyros: ");
    while ( 1 ) {
        gxf = 0.9 * gxf + 0.1 * imu_gx;
        gyf = 0.9 * gyf + 0.1 * imu_gy;
        gzf = 0.9 * gzf + 0.1 * imu_gz;
        gxs = 0.99 * gxs + 0.01 * imu_gx;
        gys = 0.99 * gys + 0.01 * imu_gy;
        gzs = 0.99 * gzs + 0.01 * imu_gz;
    
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
            Serial.println(" :)");
            break;
        }
        if (total_timer > 15000) {
            Serial.println(" :(");
            break;
        }
       delay(10);
    }
    Serial.print("Average gyros: ");
    Serial.print(gxs,4);
    Serial.print(" ");
    Serial.print(gys,4);
    Serial.print(" ");
    Serial.print(gzs,4);
    Serial.println();
}


void print_imu() {
    Serial.print(imu_gx); Serial.print(" ");
    Serial.print(imu_gy); Serial.print(" ");
    Serial.println(imu_gz);
}

