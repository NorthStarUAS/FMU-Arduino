#include <Arduino.h>
#include <SD.h>
// #include <HardwareSerial.h>

#include "setup_board.h"        // #include this early

#include "src/airdata.h"
#include "src/comms/comms_mgr.h"
#include "src/config.h"
#include "src/gps.h"
#include "src/sensors/imu_mgr.h"
#include "src/led.h"
#include "src/nav/nav_mgr.h"
#include "src/sensors/pilot.h"
#include "src/power.h"
#include "src/props2.h"
#include "src/sensors/sbus/sbus.h"

// Controls and Actuators
// uint8_t test_pwm_channel = -1; fixme not needed here?

// force/hard-code a specific board config if desired
void force_config_aura3() {
    Serial.println("Forcing an aura v2 eeprom config");
    config.board.board = 1;    // 0 = marmot v1, 1 = aura v2
    imu_mgr.defaults_aura3();
    airdata.defaults_aura3();
    led.defaults_aura3();
    config.power.have_attopilot = true;
    // pwm.act_gain_defaults();  fixme?
    pilot.init();
    config.stab.sas_rollaxis = true;
    config.stab.sas_pitchaxis = true;
    config.stab.sas_yawaxis = true;
    config.stab.sas_rollgain = 0.2;
    config.stab.sas_pitchgain = 0.2;
    config.stab.sas_yawgain = 0.2;
    config.ekf.select = message::enum_nav::none;
    // config.write_eeprom();
}

// force/hard-code a specific board config if desired
void force_config_goldy3() {
    Serial.println("Forcing a bfs/marmot eeprom config");
    config.board.board = 0;    // 0 = marmot v1, 1 = aura v2
    imu_mgr.defaults_goldy3();
    airdata.defaults_goldy3();
    led.defaults_goldy3();
    // pwm.act_gain_defaults();  fixme?
    pilot.init();
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
    imu_mgr.defaults_goldy3();
    led.defaults_goldy3();
    // pwm.act_gain_defaults();  fixme?
    pilot.init();
    config.power.have_attopilot = false;
}

static PropertyNode config_node;
static PropertyNode config_nav_node;
static PropertyNode pilot_node;
static PropertyNode status_node;

static comms_mgr_t comms_mgr;

void setup() {
    Serial.begin(115200);
    delay(1000);  // hopefully long enough for serial to come alive

    // make it different random each time
    randomSeed(analogRead(0));

    printf("\nNorthStar FMU: Rev %d\n", FIRMWARE_REV);
    printf("You are seeing this message on the usb interface.\n");
    printf("Sensor/config communication is on Serial1 @ %d baud (N81) no flow control.\n", HOST_BAUD);

    // load config from SD card
    if ( !SD.begin(BUILTIN_SDCARD)) {
        printf("Problem initializing builtin SD card ... failed.");
    } else {
        printf("SD card initialized.");
    }

    config.init();
    if ( !config.load_json_config() ) {
        config.reset_defaults();
    }

    // The following code (when enabled) will force setting a specific
    // device serial number when the device boots:
    if ( false ) {
        config.set_serial_number(118);
    }
    config.read_serial_number();
    printf("Serial Number: %d\n", config.read_serial_number());
    delay(100);

    // if ( !config.read_eeprom() ) {
    //     Serial.println("Resetting eeprom to default values.");
    //     reset_config_defaults();
    //     config.write_eeprom();
    // } else {
    //     Serial.println("Successfully loaded eeprom config.");
    // }

    // after config.init()
    config_node = PropertyNode("/config");
    config_nav_node = PropertyNode("/config/nav");
    pilot_node = PropertyNode("/pilot");
    status_node = PropertyNode("/status");

    status_node.setUInt("firmware_rev", FIRMWARE_REV);
    status_node.setUInt("master_hz", MASTER_HZ);
    status_node.setUInt("baud", TELEMETRY_BAUD);
    status_node.setUInt("serial_number", config_node.getUInt("serial_number"));

    // force/hard-code a specific board config if desired
    // force_config_aura3();
    // force_config_goldy3();

    // update imu strapdown and mag_affine matrices from config
    imu_mgr.set_strapdown_calibration();
    imu_mgr.set_mag_calibration();

    // initialize the IMU
    imu_mgr.setup();
    delay(100);

    // initialize the SBUS receiver
    sbus.setup();

    // initialize the pilot interface (RC in, out & mixer)
    pilot.init();

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
    nav_mgr.init();

    comms_mgr.init();

    Serial.println("Ready and transmitting...");
}

// main arduino loop -- Fixme: set this up on a hardware timer so the main loop can do non-time sensitive stuff, but caution on race conditions
void loop() {
    static elapsedMillis mainTimer = 0;
    static elapsedMillis hbTimer = 0;
    static elapsedMillis debugTimer = 0;

    // When new IMU data is ready (new pulse from IMU), go out and grab the IMU data
    // and output fresh IMU message plus the most recent data from everything else.
    if ( mainTimer >= DT_MILLIS ) {
        mainTimer -= DT_MILLIS;

        // fixme: add back counter for main loop timer misses ... one of those
        // things that should never happen, but if it does we want to know right
        // away.
        // if ( mainTimer > 0 ) {
        //     comms.main_loop_timer_misses++;
        //     if ( comms.main_loop_timer_misses % 25 == 0 ) {
        //         Serial.println("WARNING: main loop is not completing on time!");
        //     }
        // }

        // top priority, used for timing sync downstream.
        imu_mgr.update();

        // suck in any available gps messages
        gps.update();

        // 3. Estimate location and attitude
        if ( config_nav_node.getString("selected") != "none" ) {
            nav_mgr.update();
        }

        // fixme: check, but this should be handled down in the comms level code now.
        // // output keyed off new IMU data
        // comms.output_counter += comms.write_pilot_in_bin();
        // comms.output_counter += comms.write_gps_bin();
        // comms.output_counter += comms.write_airdata_bin();
        // comms.output_counter += comms.write_power_bin();
        // // do a little extra dance with the return value because
        // // write_status_info_bin() can reset comms.output_counter (but
        // // that gets ignored if we do the math in one step)
        // uint8_t result = comms.write_status_info_bin();
        // comms.output_counter += result;
        // if ( config.ekf.select != message::enum_nav::none ) {
        //     comms.output_counter += comms.write_nav_bin();
        // }
        // // write imu message last: used as an implicit end of data
        // // frame marker.
        // comms.output_counter += comms.write_imu_bin();

        // fixme: also check this is moved to comms_mgr
        // one minute heartbeat output
        // if ( hbTimer >= 60000 && imu_mgr.gyros_calibrated == 2) {
        //     hbTimer = 0;
        //     comms.write_status_info_ascii();
        //     comms.write_power_ascii();
        //     Serial.println();
        // }
        // // 10hz human debugging output, but only after gyros finish calibrating
        // if ( debugTimer >= 100 && imu_mgr.gyros_calibrated == 2) {
        //     debugTimer = 0;
        //     // write_pilot_in_ascii();
        //     // write_actuator_out_ascii();
        //     // comms.write_gps_ascii();
        //     // if ( config.ekf.select != message::enum_nav::none ) {
        //     //     comms.write_nav_ascii();
        //     // }
        //     // comms.write_airdata_ascii();
        //     // write_status_info_ascii();
        //     // write_imu_ascii();
        // }

        // FIXME: move this functionality to pilot.cpp?
        // uncomment this next line to test drive individual servo channels
        // (for debugging or validation.)
        // test_pwm_channel = -1;  // zero is throttle so be careful!
        // if ( test_pwm_channel >= 0 ) {
        //     pwm.write(test_pwm_channel);
        // }

        // poll the pressure sensors
        airdata.update();

        // read power values
        power.update();

        if ( pilot.read() ) {
            bool ap_state = pilot_node.getBool("ap_enabled");
            static bool last_ap_state = ap_state;
            if ( ap_state and !last_ap_state ) {
                printf("ap enabled\n");
            } else if ( !ap_state and last_ap_state ) {
                printf("ap disabled (manaul flight)\n");
            }
            last_ap_state = ap_state;
        }

        // fixme think about order and timing, but inner loop commands aren't comming from host any more so maybe less important?
        pilot.write();

        comms_mgr.update();
    }

    // keep processing while there is data in the uart buffer
    // while ( sbus.process() ) {
    //     static bool last_ap_state = pilot.ap_enabled();
    //     pilot.update_manual();
    //     if ( pilot.ap_enabled() ) {
    //         if ( !last_ap_state ) { Serial.println("ap enabled"); }
    //         mixer.update( pilot.ap_inputs );
    //     } else {
    //         if ( last_ap_state ) { Serial.println("ap disabled (manaul flight)"); }
    //         mixer.update( pilot.manual_inputs );
    //     }
    //     pwm.update();
    //     last_ap_state = pilot.ap_enabled();
    // }

    // blink the led on boards that support it
    led.update(imu_mgr.gyros_calibrated, gps.gps_data.fixType);
}
