#include <Arduino.h>
#include <SD.h>
// #include <HardwareSerial.h>

#include "setup_board.h"        // #include this early

#include "src/sensors/airdata_mgr.h"
#include "src/comms/comms_mgr.h"
#include "src/config.h"
#include "src/control/control_mgr.h"
#include "src/sensors/gps_mgr.h"
#include "src/sensors/imu_mgr.h"
#include "src/led.h"
#include "src/nav/nav_mgr.h"
#include "src/sensors/pilot.h"
#include "src/sensors/power.h"
#include "src/props2.h"
#include "src/sensors/sbus/sbus.h"
#include "src/state/state_mgr.h"

// Controls and Actuators
// uint8_t test_pwm_channel = -1; fixme not needed here?

static PropertyNode config_node;
static PropertyNode config_nav_node;
static PropertyNode pilot_node;
static PropertyNode status_node;

static comms_mgr_t comms_mgr;

void setup() {
    Serial.begin(115200);
    delay(1000);  // hopefully long enough for serial to come alive

    // different random each run
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
        printf("No config file loaded, we cannot do much without it.");
        delay(5000);
    }

    // The following code (when enabled) will force setting a specific
    // device serial number when the device boots:
    if ( false ) {
        config.set_serial_number(118);
    }
    config.read_serial_number();
    printf("Serial Number: %d\n", config.read_serial_number());
    delay(100);

    // after config.init()
    config_node = PropertyNode("/config");
    config_nav_node = PropertyNode("/config/nav");
    pilot_node = PropertyNode("/pilot");
    status_node = PropertyNode("/status");

    status_node.setUInt("firmware_rev", FIRMWARE_REV);
    status_node.setUInt("master_hz", MASTER_HZ);
    status_node.setUInt("baud", TELEMETRY_BAUD);
    status_node.setUInt("serial_number", config_node.getUInt("serial_number"));

    // initialize the IMU and calibration matrices
    imu_mgr.setup();
    imu_mgr.set_strapdown_calibration();
    imu_mgr.set_accel_calibration();
    imu_mgr.set_mag_calibration();
    delay(100);

    // initialize the SBUS receiver
    sbus.setup();

    // initialize the pilot interface (RC in, out & mixer)
    pilot.init();

    // initialize the gps receiver
    gps_mgr.setup();

    // initialize air data (marmot v1)
    airdata_mgr.setup();

    // power sensing
    analogReadResolution(16);   // set up ADC0
    power.setup();

    // led for status blinking if defined
    led.setup();

    // ekf init (just prints availability status)
    nav_mgr.init();

    // additional derived/computed/estimated values
    state_mgr.init();

    comms_mgr.init();

    Serial.println("Ready and transmitting...");
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
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
        if ( config_nav_node.getString("selected") != "none" ) {
            nav_mgr.update();
        }

        // poll the pressure sensors
        airdata_mgr.update();

        state_mgr.update(1.0 / MASTER_HZ);

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

        // status
        status_node.setUInt("available_memory", freeMemory());

        // blink the led on boards that support it
        led.update(imu_mgr.gyros_calibrated);

        comms_mgr.update();
    }

}
