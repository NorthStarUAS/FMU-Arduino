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
#include "src/sensors/sensor_mgr.h"
#include "src/state/state_mgr.h"
#include "src/util/myprof.h"

myprofile main_prof;

FS *datafs = NULL;
LittleFS_Program progmfs;

comms_mgr_t *comms_mgr;

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

    config = new config_t();
    if ( !config->load_json_config() ) {
        printf("No config file loaded, we cannot do much without it.\n");
        delay(5000);
    }

    // call this very early (before any other property tree access), but after
    // the config file is loaded.
    PropertyNodes_init();

    // The following code (when enabled) will force setting a specific
    // device serial number when the device boots:
    if ( false ) {
        config->set_serial_number(124);
    }
    config->read_serial_number();
    printf("Serial Number: %d\n", config->read_serial_number());
    delay(100);

    status_node.setUInt("firmware_rev", FIRMWARE_REV);
    status_node.setUInt("master_hz", MASTER_HZ);
    status_node.setUInt("baud", TELEMETRY_BAUD);
    status_node.setUInt("serial_number", config_node.getUInt("serial_number"));

    sensor_mgr = new sensor_mgr_t();
    sensor_mgr->init();

    // led for status blinking if defined
    led.init();

    // ekf init (just prints availability status)
    nav_mgr = new nav_mgr_t();
    nav_mgr->init();

    // additional derived/computed/estimated values
    state_mgr.init();

    fcs_mgr = new fcs_mgr_t();
    fcs_mgr->init();

    comms_mgr = new comms_mgr_t();
    comms_mgr->init();

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

        sensor_mgr->update();

        // 3. Estimate location and attitude
        nav_mgr->update();

        state_mgr.update(1.0 / MASTER_HZ);

        fcs_mgr->update(DT_MILLIS/1000.0);

        inceptors.write(); // fixme: this should become effectors after we move switches to be owned by inceptors

        // status
        status_node.setUInt("available_memory", freeram());

        // blink the led on boards that support it
        led.update(sensor_mgr->imu_mgr.gyros_calibrated);

        comms_mgr->update();

        MTP.loop();
    }
}
