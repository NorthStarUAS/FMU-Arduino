#include <HardwareSerial.h>

#include "UBLOX.h"
#include "config.h"

// IMU
volatile bool new_imu_data = false;
int gyros_calibrated = 0; // 0 = uncalibrated, 1 = calibration in progress, 2 = calibration finished
float imu_calib[10]; // the 'safe' and calibrated version of the imu sensors
int16_t imu_packed[10]; // calibrated and packed version of the imu sensors

unsigned long imu_micros = 0;

// Controls and Actuators
float receiver_norm[SBUS_CHANNELS];
float autopilot_norm[SBUS_CHANNELS];
float actuator_norm[SBUS_CHANNELS];
uint16_t actuator_pwm[PWM_CHANNELS];

// GPS
bool new_gps_data = false;
UBLOX gps(&Serial3); // ublox m8n
nav_pvt gps_data;

// COMS
bool binary_output = false; // start with ascii output (then switch to binary if we get binary commands in
unsigned long output_counter = 0;
unsigned long write_millis = 0;
int LED = 13;


void setup() {
    // put your setup code here, to run once:
    
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
  
    Serial.begin(DEFAULT_BAUD);
    delay(600); // needed delay before attempting to print anything
    
    Serial.print("\nAura Sensors: Rev "); Serial.println(FIRMWARE_REV);
    
    // The following code (when enabled) will force setting a specific device serial number.
    // set_serial_number(108);
    read_serial_number();
    
    if ( /* true || */ !config_read_eeprom() ) {
        config_load_defaults();
        config_write_eeprom();
    }

    // Serial.print("F_CPU: "); Serial.println(F_CPU);
    // Serial.print("F_PLL: "); Serial.println(F_PLL);
    // Serial.print("BAUD2DIV: "); Serial.println(BAUD2DIV(115200));
    
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
  
    // initialize comms channel write stats timer
    write_millis = millis();
}

void loop() {
    // put your main code here, to run repeatedly:
    static elapsedMillis myTimer = 0;
    static elapsedMillis airdataTimer = 0;
    static elapsedMillis blinkTimer = 0;
    static int blink_rate = 100;
    static bool blink_state = true;
    
    // When new IMU data is ready (new pulse from IMU), go out and grab the IMU data
    // and output fresh IMU message plus the most recent data from everything else.
    if ( new_imu_data ) {
        cli();
        new_imu_data = false;
        sei();
        update_imu();

        // output keyed off new IMU data
        if ( binary_output ) {
            output_counter += write_pilot_in_bin();
            output_counter += write_gps_bin();
            //output_counter += write_baro_bin();
            //output_counter += write_analog_bin();
            // do a little extra dance with the return value because write_status_info_bin()
            // can reset output_counter (but that gets ignored if we do the math in one step)
            uint8_t result = write_status_info_bin();
            output_counter += result;
            output_counter += write_imu_bin(); // write IMU data last as an implicit 'end of data frame' marker.
        } else {
            // 10hz human debugging output, but only after gyros finish calibrating
            if ( myTimer >= 100 && gyros_calibrated == 2) {
                myTimer = 0;
                // write_pilot_in_ascii();
                // write_actuator_out_ascii();
                write_gps_ascii();
                // write_baro_ascii();
                // write_analog_ascii();
                // write_status_info_ascii();
                // write_imu_ascii();
            }
        }
    }

    while ( sbus_process() ); // keep processing while there is data in the uart buffer

    if ( gps.read_ublox8() ) {
        new_gps_data = true;
        gps_data = gps.get_data();
    }

    if ( airdataTimer > 100 ) {
        airdataTimer = 0;
        // roughly 100hz airdata polling
        // airdata_update();
    }
    
    // suck in any host commmands (would I want to check for host commands
    // at a higher rate? imu rate?)
    while ( read_commands() );

    // blinking
    if ( gyros_calibrated < 2 ) {
        blink_rate = 100;
    } else if ( gps_data.fixType < 3 ) {
        blink_rate = 400;
    } else {
        blink_rate = 1000;
    }
    if ( blink_state && blinkTimer >= blink_rate*0.7 ) {
        blinkTimer = 0;
        blink_state = false;
        digitalWrite(LED, LOW);
    } else if ( !blink_state && blinkTimer >= blink_rate*0.3 ) {
        blinkTimer = 0;
        blink_state = true;
        digitalWrite(LED, HIGH);
    }
}
