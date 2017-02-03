#include <HardwareSerial.h>

#include "UBLOX.h"
#include "config.h"

// eigen test
#include <Eigen30.h>
using Eigen::MatrixXd;

#include "EKF15.h"
EKF15 ekf;

// shared with imu module
volatile bool new_imu_data = false;
volatile unsigned long int imu_counter = 0;
int gyros_calibrated = 0; // 0 = uncalibrated, 1 = calibration in progress, 2 = calibration finished
float imu_calib[10]; // the 'safe' and calibrated version of the imu sensors

float receiver_norm[MAX_CHANNELS];
float autopilot_norm[MAX_CHANNELS];
float actuator_norm[MAX_CHANNELS];
uint16_t actuator_pwm[NUM_PWM_CHANNELS];

bool new_gps_data = false;
UBLOX gps(3); // ublox m8n
gpsData uBloxData;

void setup() {
    // put your setup code here, to run once:

    Serial.begin(DEFAULT_BAUD);
    delay(500); // needed delay before attempting to print anything
    
    Serial.println("\nAura Sensors");
    
    // The following code (when enabled) will force setting a specific device serial number.
    // set_serial_number(108);
    read_serial_number();
    
    if ( !config_read_eeprom() ) {
        config_load_defaults();
        config_write_eeprom();
    }

    Serial.print("F_CPU: ");
    Serial.println(F_CPU);
    Serial.print("F_PLL: ");
    Serial.println(F_PLL);
    Serial.print("BAUD2DIV: ");
    Serial.println(BAUD2DIV(115200));
    Serial.print("Firmware Revision: ");
    Serial.println(FIRMWARE_REV);
    Serial.print("Serial Number: ");
    Serial.println(read_serial_number());
    delay(100);

    // initialize the IMU
    imu_setup();
    delay(100);

    // initialize the SBUS receiver
    sbus_setup();

    // initialize PWM output
    pwm_setup();

    // initialize the gps receiver
    gps.begin(115200);

    // Eigen
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);

    MatrixXd n = m * m;
    for ( int j = 0; j < 2; j++ ) {
      for ( int i = 0; i < 2; i++ ) {
        Serial.print(n(i,j),2); Serial.print(" ");
      }
      Serial.println();
    }
}

IMUdata imu_data;
GPSdata gps_data;
NAVdata nav_data;

void loop() {
    // put your main code here, to run repeatedly:
    
    static elapsedMillis myTimer = 0;
    if ( new_imu_data ) {
        Serial.println("new imu data");
        noInterrupts();
        new_imu_data = false;
        interrupts();
        update_imu();
        //fixme:
        imu_data.time = millis();
        imu_data.p = imu_calib[3];
        imu_data.q = imu_calib[4];
        imu_data.r = imu_calib[5];
        imu_data.ax = imu_calib[0];
        imu_data.ay = imu_calib[1];
        imu_data.az = imu_calib[2];

        static bool first_time = true;
        Serial.println("before ekf()");
        if ( first_time ) {
          first_time = false;
          nav_data = ekf.init(imu_data, gps_data);
        } else {
          nav_data = ekf.update(imu_data, gps_data);
        }
        Serial.print(nav_data.phi * 57.3); Serial.print(" ");
        Serial.print(nav_data.the * 57.3); Serial.print(" ");
        Serial.print(nav_data.psi * 57.3); Serial.println();
    }
    
    while ( sbus_process() ); // keep processing while there is data in the uart buffer

    /* look for a good GPS data packet */
    if ( gps.read(&uBloxData) ) {
        new_gps_data = true;
        gps_data.time = millis();
        gps_data.unix_sec = millis();
        gps_data.lat = uBloxData.lat;
        gps_data.lon = uBloxData.lon;
        gps_data.alt = uBloxData.hMSL;
        gps_data.vn = uBloxData.velN;
        gps_data.ve = uBloxData.velE;
        gps_data.vd = uBloxData.velD;
    }
    
    if ( myTimer > 500 ) {
        myTimer = 0;
        if ( gyros_calibrated == 2 ) {
            imu_print();
            //write_pilot_in_ascii();
            //write_gps_ascii();
        }  
    }

    unsigned long int c;
    noInterrupts();
    c = imu_counter;
    interrupts();
    Serial.print("imu:"); Serial.println(c);
    delay(20);
}

