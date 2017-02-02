#include <HardwareSerial.h>

#include "config.h"

// shared with imu module
volatile bool new_imu_data = false;
int gyros_calibrated = 0; // 0 = uncalibrated, 1 = calibration in progress, 2 = calibration finished


void setup() {
    // put your setup code here, to run once:
    Serial.begin(DEFAULT_BAUD);
    delay(500); // needed delay before attempting to print anything
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
    imu_setup();
    delay(100);
}

void loop() {
    static elapsedMillis myTimer = 0;
    // put your main code here, to run repeatedly:
    if ( new_imu_data ) {
        cli();
        new_imu_data = false;
        sei();
        update_imu();
    }
    if ( myTimer > 500 ) {
        myTimer = 0;
        if ( gyros_calibrated == 2 ) {
            imu_print();
        }
    }
}



