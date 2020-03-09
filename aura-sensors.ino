#include <Arduino.h>
#include <HardwareSerial.h>

#include "src/actuators.h"
#include "src/airdata.h"
#include "src/comms.h"
#include "src/config.h"
#include "src/ekf.h"
#include "src/gps.h"
#include "src/imu.h"
#include "src/led.h"
#include "src/pilot.h"
#include "src/power.h"
#include "src/pwm.h"
#include "src/sensors/sbus/sbus.h"

#include "setup_board.h"

// Controls and Actuators
uint8_t test_pwm_channel = -1;

// force/hard-code a specific board config if desired
void force_config_aura3() {
    Serial.println("Forcing an aura v2 eeprom config");
    config.master.board = 1;    // 0 = marmot v1, 1 = aura v2
    imu.defaults_aura3();
    airdata.defaults_aura3();
    led.defaults_aura3();
    power.config.have_attopilot = true;
    actuators.config.act_gain[0] = 1.0;
    actuators.config.act_gain[1] = 1.0;
    actuators.config.act_gain[2] = -1.0;
    actuators.config.act_gain[3] = 1.0;
    actuators.config.act_gain[4] = -1.0;
    actuators.config.mix_vtail = true;
    actuators.config.mix_Gve = 1.0;
    actuators.config.mix_Gvr = 1.0;
    actuators.config.mix_flaperon = true;
    actuators.config.mix_Gfa = 1.0;
    actuators.config.mix_Gff = 1.0;
    actuators.config.mix_autocoord = true;
    actuators.config.mix_Gac = 0.25;
    actuators.config.sas_rollaxis = true;
    actuators.config.sas_pitchaxis = true;
    actuators.config.sas_yawaxis = true;
    actuators.config.sas_rollgain = 0.2;
    actuators.config.sas_pitchgain = 0.2;
    actuators.config.sas_yawgain = 0.2;
}

// force/hard-code a specific board config if desired
void force_config_goldy3() {
    Serial.println("Forcing a bfs/marmot eeprom config");
    config.master.board = 0;    // 0 = marmot v1, 1 = aura v2
    imu.defaults_goldy3();
    airdata.defaults_goldy3();
    led.defaults_goldy3();
    actuators.config.act_gain[0] = 1.0;
    actuators.config.act_gain[1] = 1.0;
    actuators.config.act_gain[2] = -1.0;
    actuators.config.act_gain[3] = 1.0;
    actuators.config.act_gain[4] = -1.0;
    actuators.config.mix_vtail = true;
    actuators.config.mix_Gve = 1.0;
    actuators.config.mix_Gvr = 1.0;
    actuators.config.mix_flaperon = true;
    actuators.config.mix_Gfa = 1.0;
    actuators.config.mix_Gff = 1.0;
    actuators.config.mix_autocoord = true;
    actuators.config.mix_Gac = 0.25;
    actuators.config.sas_rollaxis = true;
    actuators.config.sas_pitchaxis = true;
    actuators.config.sas_yawaxis = true;
    actuators.config.sas_rollgain = 0.2;
    actuators.config.sas_pitchgain = 0.2;
    actuators.config.sas_yawgain = 0.2;
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
    
    if ( !config.read_eeprom() ) {
        Serial.println("Resetting eeprom to default values.");
        config.load_defaults();
        config.write_eeprom();
    } else {
        Serial.println("Successfully loaded eeprom config.");
    }
    
    Serial.print("Serial Number: ");
    Serial.println(config.read_serial_number());
    delay(100);

    // force/hard-code a specific board config if desired
    // force_config_aura_v2();
    // force_config_talon_marmot();
    
    // initialize the IMU
    imu.setup();
    delay(100);

    // initialize the SBUS receiver
    sbus.setup();

    // intialize actuators (before pwm)
    actuators.setup();
    
    // initialize PWM output
    pwm.setup(config.master.board);

    // initialize the gps receiver
    gps.setup();

    // initialize air data (marmot v1)
    airdata.setup();
    
    // power sensing
    analogReadResolution(16);   // set up ADC0
    power.setup(config.master.board);
    
    // led for status blinking if defined
    led.setup();

    Serial.println("Ready and transmitting...");
}

void loop() {
    // put your main code here, to run repeatedly:
    static elapsedMillis mainTimer = 0;
    static elapsedMillis debugTimer = 0;
       
    // When new IMU data is ready (new pulse from IMU), go out and grab the IMU data
    // and output fresh IMU message plus the most recent data from everything else.
    if ( mainTimer >= DT_MILLIS ) {
        mainTimer -= DT_MILLIS;
        
        // top priority, used for timing sync downstream.
        imu.update();

        ekf.update();
        
        // output keyed off new IMU data
        comms.output_counter += comms.write_pilot_in_bin();
        comms.output_counter += comms.write_gps_bin();
        comms.output_counter += comms.write_airdata_bin();
        comms.output_counter += comms.write_power_bin();
        comms.output_counter += comms.write_nav_bin();
        // do a little extra dance with the return value because
        // write_status_info_bin() can reset comms.output_counter (but
        // that gets ignored if we do the math in one step)
        uint8_t result = comms.write_status_info_bin();
        comms.output_counter += result;
        comms.output_counter += comms.write_imu_bin(); // write IMU data last as an implicit 'end of data frame' marker.

        // 10hz human debugging output, but only after gyros finish calibrating
        if ( debugTimer >= 100 && imu.gyros_calibrated == 2) {
            debugTimer = 0;
            // write_pilot_in_ascii();
            // write_actuator_out_ascii();
            // comms.write_gps_ascii();
            comms.write_nav_ascii();
            // write_airdata_ascii();
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
        pilot.update_manual();
        if ( pilot.ap_enabled() ) {
            actuators.update( pilot.ap_inputs );
        } else {
            actuators.update( pilot.manual_inputs );
        }
        pwm.update();
    }

    // suck in any host commmands (flight control updates, etc.)
    comms.read_commands();

    // blink the led on boards that support it
    led.update(imu.gyros_calibrated, gps.gps_data.fixType);
}
