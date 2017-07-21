#include <MPU9250.h>

/* MPU-9250 IMU */

// IMU full scale ranges, DLPF bandwidth, interrupt SRD, and interrupt pin
const uint8_t MPU9250_SRD = 9;  // Data Output Rate = 1000 / (1 + SRD)
const uint8_t MPU_CS_PIN = 24;  // SPI CS PIN
const uint8_t MPU_SYNC_PIN = 27;

const float _pi = 3.14159265358979323846;
const float _g = 9.807;
const float _d2r = _pi / 180.0;

const float _gyro_lsb_per_dps = 32767.5 / 500;  // -500 to +500 spread across 65535
const float gyroScale = _d2r / _gyro_lsb_per_dps;

const float _accel_lsb_per_dps = 32767.5 / 8;   // -4g to +4g spread across 65535
const float accelScale = _g / _accel_lsb_per_dps;

const float magScale = 0.01;
const float tempScale = 0.01;

MPU9250 IMU(MPU_CS_PIN);  // spi

// any code that reads imu_sensors_shared should protect those reads
// with cli() / sei() calls because this array is modified by an ISR
// that could be called at any time, even in the middle of a variable
// read which could lead to corrupted values.

// the volatile (shared) storage for the imu sensors
volatile float imu_sensors_shared[10];
volatile unsigned long imu_micros_shared = 0;

// the 'safe' but raw version of the imu sensors
float imu_uncalibrated[10];

float gyro_calib[3] = { 0.0, 0.0, 0.0 };


// configure the IMU settings and setup the ISR to aquire the data
void imu_setup() {
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
    
    pinMode(MPU_SYNC_PIN, INPUT);
    attachInterrupt(MPU_SYNC_PIN, dataAcquisition, RISING);
}


// ISR, system automatically disables interrupts when running
void dataAcquisition() {
    // imu data
    imu_micros_shared = micros();
    
    float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
    // double call because of command/respose pipelining
    IMU.getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
    //IMU.getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
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
    
    // onboard air data query: this is not my favorite place to put
    // this code, but because we share spi with the IMU we need to
    // sequence these or they could step on each other.
    if ( bme_status >= 0 ) {
        // get the pressure (Pa), temperature (C),
        // and humidity data (%RH) all at once
        float pressure, temp, humidity;
        bme.getData(&pressure, &temp, &humidity);
        bme_press = pressure;
        bme_temp = temp;
        bme_hum = humidity;
    }
}


// copy the dangerous 'volatile' shared version of imu data to the
// safe global copy.
void update_imu() {
    cli();
    for ( int i = 0; i < 10; i++ ) {
        imu_uncalibrated[i] = imu_sensors_shared[i];
    }
    imu_micros = imu_micros_shared;
    sei();
    
    for ( int i = 0; i < 10; i++ ) {
        imu_calib[i] = imu_uncalibrated[i];
    }
    if ( gyros_calibrated < 2 ) {
        calibrate_gyros();
    } else {
        for ( int i = 0; i < 3; i++ ) {
            imu_calib[i+3] -= gyro_calib[i];
        }
    }
    // packed imu structure
    for ( int i = 0; i < 3; i++ ) {
        imu_packed[i] = imu_calib[i] / accelScale;
    }
    for ( int i = 3; i < 6; i++ ) {
        imu_packed[i] = imu_calib[i] / gyroScale;
    }
    for (int i = 6; i < 9; i++ ) {
        imu_packed[i] = imu_calib[i] / magScale;
    }
    imu_packed[9] = imu_calib[9] / tempScale;
}


// stay alive for up to 15 seconds looking for agreement between a 1
// second low pass filter and a 0.1 second low pass filter.  If these
// agree (close enough) for 4 consecutive seconds, then we calibrate
// with the 1 sec low pass filter value.  If time expires the
// calibration fails and we run with raw gyro values.
void calibrate_gyros() {
    static float gxs = imu_uncalibrated[3];
    static float gys = imu_uncalibrated[4];
    static float gzs = imu_uncalibrated[5];
    static float gxf = imu_uncalibrated[3];
    static float gyf = imu_uncalibrated[4];
    static float gzf = imu_uncalibrated[5];
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
    
    gxf = 0.9 * gxf + 0.1 * imu_uncalibrated[3];
    gyf = 0.9 * gyf + 0.1 * imu_uncalibrated[4];
    gzf = 0.9 * gzf + 0.1 * imu_uncalibrated[5];
    gxs = 0.99 * gxs + 0.01 * imu_uncalibrated[3];
    gys = 0.99 * gys + 0.01 * imu_uncalibrated[4];
    gzs = 0.99 * gzs + 0.01 * imu_uncalibrated[5];
    
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
    if (good_timer > 4100) {
        // set gyro zero points from the 'slow' filter.
        gyro_calib[0] = gxs;
        gyro_calib[1] = gys;
        gyro_calib[2] = gzs;
        gyros_calibrated = 2;
        update_imu(); // update imu_calib values before anything else get's a chance to read them
        Serial.println(" good.");
        Serial.print("Average gyros: ");
        Serial.print(gyro_calib[0],4);
        Serial.print(" ");
        Serial.print(gyro_calib[1],4);
        Serial.print(" ");
        Serial.print(gyro_calib[2],4);
        Serial.println();
    } else if (total_timer > 15000) {
        gyros_calibrated = 2;
        Serial.println(" failed.");
    }
}
