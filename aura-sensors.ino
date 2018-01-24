#include <HardwareSerial.h>

#include "UBLOX-AuraUAS.h"
#include "config.h"

// IMU
int gyros_calibrated = 0; // 0 = uncalibrated, 1 = calibration in progress, 2 = calibration finished
float imu_calib[10]; // the calibrated version of the imu sensors
int16_t imu_packed[10]; // calibrated and packed version of the imu sensors
unsigned long imu_micros = 0;

// Controls and Actuators
float receiver_norm[SBUS_CHANNELS];
uint8_t receiver_flags = 0x00;
float autopilot_norm[SBUS_CHANNELS];
float actuator_norm[SBUS_CHANNELS];
uint16_t actuator_pwm[PWM_CHANNELS];

// GPS
UBLOX_AuraUAS gps(&Serial3); // ublox m8n
bool new_gps_data = false;
nav_pvt gps_data;

// Air Data
float airdata_staticPress_pa = 0.0;
float airdata_diffPress_pa = 0.0;

// Analog In and Battery Voltage
#if defined PIKA_V11 || defined AURA_V10
 const uint8_t pwr_pin = A0;
#elif defined MARMOT_V1
 const uint8_t pwr_pin = 15;
 const uint8_t avionics_pin = A22;
#endif

const float analogResolution = 65535.0f;
const float pwr_scale = 11.0f;
const float avionics_scale = 2.0f;
float pwr_v = 0.0;
float avionics_v = 0.0;
    
// COMS
// Serial = usb, Serial1 connects to /dev/ttyO4 on beaglebone in pika-1.1 and marmot v1 hardware
unsigned long output_counter = 0;
unsigned long write_millis = 0;
#if defined PIKA_V11 || defined AURA_V10
 int LED = 13;
 elapsedMillis blinkTimer = 0;
 unsigned int blink_rate = 100;
 bool blink_state = true;
#endif

volatile int main_loop_signal = 0;
IntervalTimer myTimer;

void setup() {
    // put your setup code here, to run once:

    Serial.begin(DEFAULT_BAUD);
    Serial1.begin(DEFAULT_BAUD);
    delay(600); // needed delay before attempting to print anything

    Serial.print("\nAura Sensors: Rev "); Serial.println(FIRMWARE_REV);
    Serial.println("You are seeing this message on the usb interface.");
    Serial.print("Sensor/config communication is on Serial1 @ ");
    Serial.print(DEFAULT_BAUD);
    Serial.println(" baud (N81) no flow control.");
    
    // The following code (when enabled) will force setting a specific device serial number.
    // set_serial_number(112);
    read_serial_number();
    
    if ( /* true || */ !config_read_eeprom() ) {
        config_load_defaults();
        config_write_eeprom();
    }
    
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
    Serial.println("PWM ready.");

    // initialize the gps receiver
    gps.begin(115200);

    // initialize air data (marmot v1)
    airdata_setup();
    
    // set up ADC0
    analogReadResolution(16);

#if defined PIKA_V11 || defined AURA_V10
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
#endif

    // initialize comms channel write stats timer
    write_millis = millis();
    
    Serial.println("Ready and transmitting...");

    myTimer.begin(genSignal, 1000000 / MASTER_HZ);
}

void genSignal() {
    main_loop_signal++;
}

void loop() {
    // put your main code here, to run repeatedly:
    static elapsedMillis debugTimer = 0;
        
    // When new IMU data is ready (new pulse from IMU), go out and grab the IMU data
    // and output fresh IMU message plus the most recent data from everything else.
    volatile bool do_data_aquisition = false;
    cli();
    if (main_loop_signal > 0) {
        do_data_aquisition = true;
        main_loop_signal--;
    }
    sei();
    
    if ( do_data_aquisition ) {
        // top priority, used for timing sync downstream.
        imu_update();
 
        // output keyed off new IMU data
        output_counter += write_pilot_in_bin();
        output_counter += write_gps_bin();
        output_counter += write_airdata_bin();
        // output_counter += write_analog_bin();
        // do a little extra dance with the return value because write_status_info_bin()
        // can reset output_counter (but that gets ignored if we do the math in one step)
        uint8_t result = write_status_info_bin();
        output_counter += result;
        output_counter += write_imu_bin(); // write IMU data last as an implicit 'end of data frame' marker.

        // 10hz human debugging output, but only after gyros finish calibrating
        if ( debugTimer >= 100 && gyros_calibrated == 2) {
            debugTimer = 0;
            // write_pilot_in_ascii();
            // write_actuator_out_ascii();
            // write_gps_ascii();
            write_airdata_ascii();
            // write_analog_ascii();
            // write_status_info_ascii();
            // write_imu_ascii();
        }

        airdata_update();

        if ( gps.read_ublox8() ) {
            new_gps_data = true;
            gps_data = gps.get_data();
        }

        // battery voltage
        uint16_t ain;
        ain = analogRead(pwr_pin);
        pwr_v = ((float)ain) * 3.3 / analogResolution * pwr_scale;

        #if defined MARMOT_V1
         // marmot v1
         ain = analogRead(avionics_pin);
         avionics_v = ((float)ain) * 3.3 / analogResolution * avionics_scale;
        #endif
    }

    while ( sbus_process() ); // keep processing while there is data in the uart buffer

    // suck in any host commmands (flight control updates, etc.)
    while ( read_commands() );

    // blink the led on boards that support it
    #if defined PIKA_V11 || defined AURA_V10
     if ( gyros_calibrated < 2 ) {
         blink_rate = 50;
     } else if ( gps_data.fixType < 3 ) {
         blink_rate = 300;
     } else {
         blink_rate = 1000;
     }
     if ( blinkTimer >= blink_rate ) {
         blinkTimer = 0;
         blink_state = !blink_state;
         digitalWrite(LED, blink_state);
     }
    #endif
}
