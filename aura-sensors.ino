#include <HardwareSerial.h>

#include "config.h"


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
    calibrate_gyros();
}

void loop() {
    // put your main code here, to run repeatedly:
    print_imu();
    delay(1000);

}


