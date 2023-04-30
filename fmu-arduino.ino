#include <Arduino.h>
#include <HardwareSerial.h>

#include "setup_board.h"        // #include this early

#include "src/airdata.h"
#include "src/comms.h"
#include "src/config.h"
#include "src/ekf.h"
#include "src/gps.h"
#include "src/imu.h"
#include "src/led.h"
#include "src/mixer.h"
#include "src/pilot.h"
#include "src/power.h"
#include "src/pwm.h"
#include "src/sensors/sbus/sbus.h"


// Controls and Actuators
uint8_t test_pwm_channel = -1;

// force/hard-code a specific board config if desired
void force_config_aura3() {
    Serial.println("Forcing an aura v2 eeprom config");
    config.board.board = 1;    // 0 = marmot v1, 1 = aura v2
    imu.defaults_aura3();
    airdata.defaults_aura3();
    led.defaults_aura3();
    config.power.have_attopilot = true;
    pwm.act_gain_defaults();
    mixer.setup();
    config.stab.sas_rollaxis = true;
    config.stab.sas_pitchaxis = true;
    config.stab.sas_yawaxis = true;
    config.stab.sas_rollgain = 0.2;
    config.stab.sas_pitchgain = 0.2;
    config.stab.sas_yawgain = 0.2;
    config.ekf.select = message::enum_nav::none;
    config.write_eeprom();
}

// force/hard-code a specific board config if desired
void force_config_goldy3() {
    Serial.println("Forcing a bfs/marmot eeprom config");
    config.board.board = 0;    // 0 = marmot v1, 1 = aura v2
    imu.defaults_goldy3();
    airdata.defaults_goldy3();
    led.defaults_goldy3();
    pwm.act_gain_defaults();
    mixer.setup();
    config.stab.sas_rollaxis = true;
    config.stab.sas_pitchaxis = true;
    config.stab.sas_yawaxis = true;
    config.stab.sas_rollgain = 0.2;
    config.stab.sas_pitchgain = 0.2;
    config.stab.sas_yawgain = 0.2;
    config.ekf.select = message::enum_nav::none;
}

void reset_config_defaults() {
    Serial.println("Setting default config ...");
    config.board.board = 0;
    imu.defaults_goldy3();
    led.defaults_goldy3();
    pwm.act_gain_defaults();
    mixer.sas_defaults();
    mixer.setup();
    config.power.have_attopilot = false;
}

void setup() {
    // put your setup code here, to run once:

    Serial.begin(DEFAULT_BAUD);
    delay(1000);  // hopefully long enough for serial to come alive

    comms.setup();

    Serial.print("\nAura Sensors: Rev "); Serial.println(FIRMWARE_REV);
    Serial.println("You are seeing this message on the usb interface.");
    Serial.print("Sensor/config communication is on Serial1 @ ");
    Serial.print(DEFAULT_BAUD);
    Serial.println(" baud (N81) no flow control.");
    
    // The following code (when enabled) will force setting a specific
    // device serial number when the device boots:
    // config.set_serial_number(116);
    config.read_serial_number();
    
    Serial.print("Serial Number: ");
    Serial.println(config.read_serial_number());
    delay(100);

    if ( !config.read_eeprom() ) {
        Serial.println("Resetting eeprom to default values.");
        reset_config_defaults();
        config.write_eeprom();
    } else {
        Serial.println("Successfully loaded eeprom config.");
    }
    
    // force/hard-code a specific board config if desired
    // force_config_aura3();
    // force_config_goldy3();
    
    // update imu strapdown and mag_affine matrices from config
    imu.set_strapdown_calibration();
    imu.set_mag_calibration();
    
    // initialize the IMU
    imu.setup();
    delay(100);

    // initialize the SBUS receiver
    sbus.setup();

    // initialize mixer (before actuators/pwm)
    mixer.setup();
    
    // initialize PWM output
    pwm.setup(config.board.board);

    // initialize the gps receiver
    gps.setup();

    // initialize air data (marmot v1)
    airdata.setup();
    
    // power sensing
    analogReadResolution(16);   // set up ADC0
    power.setup(config.board.board);
    
    // led for status blinking if defined
    led.setup();

    // ekf init (just prints availability status)
    ekf.setup();
    
    Serial.println("Ready and transmitting...");
}

// main arduino loop
void loop() {
    static elapsedMillis mainTimer = 0;
    static elapsedMillis hbTimer = 0;
    static elapsedMillis debugTimer = 0;
       
    // When new IMU data is ready (new pulse from IMU), go out and grab the IMU data
    // and output fresh IMU message plus the most recent data from everything else.
    if ( mainTimer >= DT_MILLIS ) {
        mainTimer -= DT_MILLIS;

        if ( mainTimer > 0 ) {
            comms.main_loop_timer_misses++;
            if ( comms.main_loop_timer_misses % 25 == 0 ) {
                Serial.println("WARNING: main loop is not completing on time!");
            }
        }
        
        // top priority, used for timing sync downstream.
        imu.update();

        if ( config.ekf.select != message::enum_nav::none ) {
            ekf.update();
        }
        
        // output keyed off new IMU data
        comms.output_counter += comms.write_pilot_in_bin();
        comms.output_counter += comms.write_gps_bin();
        comms.output_counter += comms.write_airdata_bin();
        comms.output_counter += comms.write_power_bin();
        // do a little extra dance with the return value because
        // write_status_info_bin() can reset comms.output_counter (but
        // that gets ignored if we do the math in one step)
        uint8_t result = comms.write_status_info_bin();
        comms.output_counter += result;
        if ( config.ekf.select != message::enum_nav::none ) {
            comms.output_counter += comms.write_nav_bin();
        }
        // write imu message last: used as an implicit end of data
        // frame marker.
        comms.output_counter += comms.write_imu_bin();

        // one minute heartbeat output
        if ( hbTimer >= 60000 && imu.gyros_calibrated == 2) {
            hbTimer = 0;
            comms.write_status_info_ascii();
            comms.write_power_ascii();
            Serial.println();
        }
        // 10hz human debugging output, but only after gyros finish calibrating
        if ( debugTimer >= 100 && imu.gyros_calibrated == 2) {
            debugTimer = 0;
            // write_pilot_in_ascii();
            // write_actuator_out_ascii();
            // comms.write_gps_ascii();
            // if ( config.ekf.select != message::enum_nav::none ) {
            //     comms.write_nav_ascii();
            // }
            // comms.write_airdata_ascii();
            // write_status_info_ascii();
            // write_imu_ascii();
        }

        // uncomment this next line to test drive individual servo channels
        // (for debugging or validation.)
        test_pwm_channel = -1;  // zero is throttle so be careful!
        if ( test_pwm_channel >= 0 ) {
            pwm.update(test_pwm_channel);
        }

        // poll the pressure sensors
        airdata.update();

        // read power values
        power.update();

        // suck in any available gps messages
        gps.update();
    }
    
    // keep processing while there is data in the uart buffer
    while ( sbus.process() ) {
        static bool last_ap_state = pilot.ap_enabled();
        pilot.update_manual();
        if ( pilot.ap_enabled() ) {
            if ( !last_ap_state ) { Serial.println("ap enabled"); }
            mixer.update( pilot.ap_inputs );
        } else {
            if ( last_ap_state ) { Serial.println("ap disabled (manaul flight)"); }
            mixer.update( pilot.manual_inputs );
        }
        pwm.update();
        last_ap_state = pilot.ap_enabled();
    }

    // suck in any host commmands (flight control updates, etc.)
    comms.read_commands();

    // blink the led on boards that support it
    led.update(imu.gyros_calibrated, gps.gps_data.fixType);
}
