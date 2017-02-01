#include <MPU9250.h>

/* MPU-9250 IMU */
MPU9250 IMU(0x68, 0);

// IMU full scale ranges, DLPF bandwidth, interrupt SRD, and interrupt pin
const mpu9250_accel_range MPU_9250_ACCEL_RANGE = ACCEL_RANGE_8G;
const mpu9250_gyro_range MPU_9250_GYRO_RANGE = GYRO_RANGE_250DPS;
const mpu9250_dlpf_bandwidth MPU9250_BANDWIDTH = DLPF_BANDWIDTH_41HZ;
const uint8_t MPU9250_SRD = 9;  // Data Output Rate = 1000 / (1 + SRD)
const uint8_t SYNC_PIN = 2;

volatile float imu_sensors[10];

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
    imu_sensors[0] = ax;
    imu_sensors[1] = ay;
    imu_sensors[2] = az;
    imu_sensors[3] = gx;
    imu_sensors[4] = gy;
    imu_sensors[5] = gz;
    imu_sensors[6] = hx;
    imu_sensors[7] = hy;
    imu_sensors[8] = hz;
    imu_sensors[9] = t;
    sei();
}


void calibrate_gyros() {
    float gxs = imu_sensors[3];
    float gys = imu_sensors[4];
    float gzs = imu_sensors[5];
    float gxf = imu_sensors[3];
    float gyf = imu_sensors[4];
    float gzf = imu_sensors[5];
    const float cutoff = 0.005;
    elapsedMillis total_timer = 0;
    elapsedMillis good_timer = 0;
    elapsedMillis output_timer = 0;

    Serial.print("Calibrating gyros: ");
    while ( true ) {
        gxf = 0.9 * gxf + 0.1 * imu_sensors[3];
        gyf = 0.9 * gyf + 0.1 * imu_sensors[4];
        gzf = 0.9 * gzf + 0.1 * imu_sensors[5];
        gxs = 0.99 * gxs + 0.01 * imu_sensors[3];
        gys = 0.99 * gys + 0.01 * imu_sensors[4];
        gzs = 0.99 * gzs + 0.01 * imu_sensors[5];
    
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
    for ( int i = 0; i < 6; i++ ) {
        Serial.print(imu_sensors[i],4); Serial.print(" ");
    }
    Serial.println(imu_sensors[9],2);
}

