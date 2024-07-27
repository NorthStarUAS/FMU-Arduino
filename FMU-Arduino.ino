#include <Arduino.h>
#include <SD.h>
#include <LittleFS.h>
#include <MTP_Teensy.h>

#include "setup_board.h"        // #include this early
#include "src/nodes.h"

#include "src/comms/comms_mgr.h"
#include "src/config.h"
#include "src/fcs/fcs_mgr.h"
#include "src/led.h"
#include "src/nav/nav_mgr.h"
#include "src/sensors/airdata_mgr.h"
#include "src/sensors/gps_mgr.h"
#include "src/sensors/imu_mgr.h"
#include "src/sensors/pilot.h"
#include "src/sensors/power.h"
#include "src/sensors/sbus/sbus.h"
#include "src/state/state_mgr.h"
#include "src/util/myprof.h"

myprofile main_prof;

// Controls and Actuators
// uint8_t test_pwm_channel = -1; fixme not needed here?

FS *datafs = NULL;
LittleFS_Program progmfs;

static comms_mgr_t comms_mgr;

void setup() {
    Serial.begin(115200);
    delay(1000);  // hopefully long enough for serial to come alive

    // different random seed each run
    randomSeed(analogRead(0));

    printf("\nNorthStar FMU: Rev %d\n", FIRMWARE_REV);
    printf("You are seeing this message on the usb interface.\n");
    printf("Sensor/config communication is on Serial1 @ %d baud (N81) no flow control.\n", HOST_BAUD);

    // initialize SD card
    if ( !SD.begin(BUILTIN_SDCARD)) {
        printf("Cannot initializing builtin SD card ... no card?\n");
    } else {
        printf("SD card initialized for config and logging.\n");
        datafs = &SD;
        MTP.addFilesystem(SD, "SD Card");
    }

    // initialize onboard flash file storage
    uint32_t lfs_progm_bytes = 1024*1024;   // allocate 1Mb flash disk, doesn't seem to work if we try to allocate larger even though we should be able to do 7+ Mb
    if ( !progmfs.begin(lfs_progm_bytes) ) {
        printf("Problem initializing flash storage ... failed.\n");
    } else {
        printf("Program memory flash initialized: %lu bytes.\n", lfs_progm_bytes);
        if ( datafs == NULL ) {
            printf("Using progm for config and logging.\n");
            datafs = &progmfs;
        }
        MTP.addFilesystem(progmfs, "Program Memory Flash");
    }

    MTP.begin();

    if ( !config.load_json_config() ) {
        printf("No config file loaded, we cannot do much without it.\n");
        delay(5000);
    }

    // call this very early (before any other property tree access), but after
    // the config file is loaded.
    PropertyNodes_init();

    // The following code (when enabled) will force setting a specific
    // device serial number when the device boots:
    if ( false ) {
        config.set_serial_number(124);
    }
    config.read_serial_number();
    printf("Serial Number: %d\n", config.read_serial_number());
    delay(100);

    status_node.setUInt("firmware_rev", FIRMWARE_REV);
    status_node.setUInt("master_hz", MASTER_HZ);
    status_node.setUInt("baud", TELEMETRY_BAUD);
    status_node.setUInt("serial_number", config_node.getUInt("serial_number"));

    // initialize the IMU and calibration matrices
    imu_mgr.init();
    imu_mgr.set_strapdown_calibration();
    imu_mgr.set_accel_calibration();
    imu_mgr.set_mag_calibration();
    delay(100);

    // initialize the SBUS receiver
    sbus.init();

    // initialize the pilot interface (RC in, out & mixer)
    pilot.init();

    // initialize the gps receiver
    gps_mgr.init();

    // initialize air data (marmot v1)
    airdata_mgr.init();

    // power sensing
    analogReadResolution(16);   // set up ADC0
    power.init();

    // led for status blinking if defined
    led.init();

    // ekf init (just prints availability status)
    nav_mgr.init();

    // additional derived/computed/estimated values
    state_mgr.init();

    comms_mgr.init();

    Serial.println("Ready and transmitting...");
}

extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern char *__brkval;
int freeram() {
  return (char *)&_heap_end - __brkval;
}

// main arduino loop -- Fixme: set this up on a hardware timer so the main loop can do non-time sensitive stuff, but caution on race conditions
void loop() {
    static elapsedMillis mainTimer = 0;
    // static elapsedMillis hbTimer = 0;
    // static elapsedMillis debugTimer = 0;

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

        // 1. Sense motion (top priority, used for timing sync downstream.)
        imu_mgr.update();

        // 2. Check for gps updates
        gps_mgr.update();

        // 3. Estimate location and attitude
        nav_mgr.update();

        // poll the pressure sensors
        airdata_mgr.update();

        state_mgr.update(1.0 / MASTER_HZ);

        if ( !status_node.getBool("HIL_mode") ) {
            // read power values
            power.update();
        }

        if ( pilot.read() ) {
            bool ap_state = inceptors_node.getBool("ap_enabled");
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

        // status
        status_node.setUInt("available_memory", freeram());

        // blink the led on boards that support it
        led.update(imu_mgr.gyros_calibrated);

        comms_mgr.update();

        MTP.loop();
    }

}
