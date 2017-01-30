#include <HardwareSerial.h>

#include <MPU9250.h>

#include "config.h"


/* MPU-9250 IMU */
MPU9250 IMU(0x68, 0);

// IMU full scale ranges, DLPF bandwidth, interrupt SRD, and interrupt pin
const mpu9250_accel_range MPU_9250_ACCEL_RANGE = ACCEL_RANGE_8G;
const mpu9250_gyro_range MPU_9250_GYRO_RANGE = GYRO_RANGE_250DPS;
const mpu9250_dlpf_bandwidth MPU9250_BANDWIDTH = DLPF_BANDWIDTH_41HZ;
const uint8_t MPU9250_SRD = 9;  // Data Output Rate = 1000 / (1 + SRD)
const uint8_t SYNC_PIN = 2;

volatile float imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz, imu_hx, imu_hy, imu_hz, imu_t;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(DEFAULT_BAUD);
    delay(500);
    Serial.println("\nAura Sensors");
    
    // The following code (when enabled) will force setting a specific device serial number.
    set_serial_number(108);
    read_serial_number();
    
    if ( !config_read_eeprom() ) {
        config_load_defaults();
        config_write_eeprom();
    }
    
    Serial.print("Firmware Revision: ");
    Serial.println(FIRMWARE_REV);
    Serial.print("Serial Number: ");
    Serial.println(read_serial_number());
    delay(100);

    // initialize the IMU
    if(IMU.begin(MPU_9250_ACCEL_RANGE,MPU_9250_GYRO_RANGE) < 0){}
    
    // set the DLPF and interrupts
    if(IMU.setFilt(MPU9250_BANDWIDTH,MPU9250_SRD) < 0){}
    pinMode(SYNC_PIN,INPUT);
    attachInterrupt(SYNC_PIN, dataAcquisition, RISING);
}

void loop() {
    // put your main code here, to run repeatedly:
    Serial.print(imu_gx); Serial.print(" ");
    Serial.print(imu_gy); Serial.print(" ");
    Serial.println(imu_gz);
    delay(1000);

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

