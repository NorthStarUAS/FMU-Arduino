#include <Arduino.h>
#include <SD.h>
#include <LittleFS.h>
#include <MTP_Teensy.h>

#include "setup_board.h"        // #include this early
#include "src/nodes.h"

#include "src/config.h"
#include "src/led.h"
#include "src/comms/comms_mgr.h"
#include "src/fcs/fcs_mgr.h"
#include "src/mission/mission_mgr.h"
#include "src/nav/nav_mgr.h"
#include "src/sensors/sensor_mgr.h"
#include "src/environment/env_mgr.h"
#include "src/util/freeram.h"
#include "src/util/profile.h"

IntervalTimer main_scheduler;

FS *configfs = nullptr;
FS *logfs = nullptr;
LittleFS_Program progmfs;

// fixme: faster init/bootup?
// fixme: add wdt ... because chute happens, over temp, external interference, sensor/wiring issue, etc.

void setup() {
    Serial.begin(115200);
    uint32_t timeout = millis() + 1000;
    // wait for up to a second for Serial to become ready (continue if not ready!)
    while ( not Serial and millis() < timeout ) {
        delay(1);
    }

    // different random seed each run
    randomSeed(analogRead(0));

    printf("\nNorthStar FMU: Rev %d\n\n", FIRMWARE_REV);
    // printf("You are seeing this message on the usb interface.\n");
    // printf("Sensor/config communication is on Serial1 @ %d baud (N81) no flow control.\n", HOST_BAUD);

    // initialize onboard flash file storage
    if ( lfs_progm_bytes == 0 ) {
        // not supported
    } else if ( not progmfs.begin(lfs_progm_bytes) ) {
        printf("Unable to initialize flash storage ... failed to allocate %lu bytes.\n", lfs_progm_bytes);
    } else {
        printf("Program memory flash initialized: %lu bytes.\n", lfs_progm_bytes);
        printf("Using progm for config files.\n");
        configfs = &progmfs;
        MTP.addFilesystem(progmfs, "Program Memory Flash");
    }

    // initialize SD card
    // SD.begin(BUILTIN_SDCARD)
    if ( not SD.sdfs.begin(SdioConfig(DMA_SDIO))) {
        printf("Cannot initializing builtin SD card ... no card?\n");
    } else {
        printf("SD card initialized for logging.\n");
        logfs = &SD;
        if ( configfs == nullptr ) {
            configfs = &SD;
            printf("Using SD card for config files.\n");
        }
        MTP.addFilesystem(SD, "SD Card");
    }

    MTP.begin();

    config = new config_t();
    if ( configfs == nullptr or not config->load_json_config() ) {
        printf("No config file loaded, we cannot do much without it.\n");
        delay(5000);
    }

    // call this very early (before any other property tree access), but after
    // the config file is loaded.
    PropertyNodes_init();

    // The following code (when enabled) will force setting a specific
    // device serial number when the device boots:
    if ( false ) {
        config->set_serial_number(125);
    }
    config->read_serial_number();
    printf("Serial Number: %d\n", config->read_serial_number());

    status_node.setUInt("firmware_rev", FIRMWARE_REV);
    status_node.setUInt("master_hz", MASTER_HZ);
    // status_node.setUInt("baud", TELEMETRY_BAUD);
    status_node.setUInt("serial_number", config_node.getUInt("serial_number"));

    sensor_mgr = new sensor_mgr_t();
    sensor_mgr->init();

    // led for status blinking if defined
    led.init();

    // ekf init (just prints availability status)
    nav_mgr = new nav_mgr_t();
    nav_mgr->init();

    // additional derived/computed/estimated values
    env_mgr.init();

    fcs_mgr = new fcs_mgr_t();
    fcs_mgr->init();

    mission_mgr = new mission_mgr_t();
    mission_mgr->init();

    comms_mgr = new comms_mgr_t();
    comms_mgr->init();

    Serial.print("Boot seconds: "); Serial.println(millis()/1000.0, 2);
    Serial.println("Ready and transmitting..."); Serial.println();

    // Start the main data collection loop on a hardware interval timer
    main_scheduler.begin(main_loop, 1000000/MASTER_HZ);
}

void main_loop() {
    static const float dt = 1.0 / MASTER_HZ;
    static unsigned int counter = 0;
    static unsigned int overruns = 0;

    main_prof.start();

    // fixme: add back counter for main loop timer misses ... one of those
    // things that should never happen, but if it does we want to know right
    // away.
    // if ( mainTimer > 0 ) {
    //     comms.main_loop_timer_misses++;
    //     if ( comms.main_loop_timer_misses % 25 == 0 ) {
    //         Serial.println("WARNING: main loop is not completing on time!");
    //     }
    // }

    // 1. Aviate: This is the core "sense, compute, response" block.  We want
    //    this to run at a perfect update interval with the least possible
    //    transport delay.
    sensor_mgr->update(dt);
    nav_mgr->update();
    env_mgr.update(dt);
    fcs_mgr->update(dt);

    // 2. Navigate: These are the higher level tasks and objectives
    mission_mgr->update(dt);

    // 3. Communicate
    status_node.setUInt("available_memory", freeram());
    led.update();
    comms_mgr->update();

    uint32_t elapsed_micros = main_prof.stop();

    if ( elapsed_micros > DT_MILLIS * 1000 ) {
        overruns++;
    }
    counter++;
    if ( counter % 1000 == 0 ) {
        Serial.print("Main loop overruns: "); Serial.println(overruns);
        profile_node.setUInt("main_loop_overrruns", overruns);
        profile_print_stats();
        profile_to_props();
    }
}

void loop() {
    // These things run at lower priority outside the interrupt handler. (But
    // absolutely no property tree access here!)
    MTP.loop();
    comms_mgr->data_logger.write_buffer();
    delay(1);
}