#include "sensor_mgr.h"

void sensor_mgr_t::init() {
    // initialize air data (marmot v1)
    airdata_mgr.init();

    // initialize the gps receiver
    gps_mgr.init();

    // initialize the IMU and calibration matrices
    imu_mgr.init();
    imu_mgr.set_strapdown_calibration();
    imu_mgr.set_accel_calibration();
    imu_mgr.set_mag_calibration();

    // power sensing
    analogReadResolution(16);   // set up ADC0
    power.init();

    // initialize the pilot interface (RC in, out & mixer)
    inceptors.init();

    hil_testing_node.pretty_print();
    printf("HIL: %d\n", hil_testing_node.getBool("enable"));
    if ( hil_testing_node.getBool("enable") ) {
        printf("NOTE: HIL mode enabled.");
        if ( hil_testing_node.getString("inceptors") == "rc" ) {
            printf("  Inceptor input from RC.");
        }
        printf("\n");
    } else {
        printf("Normal (real) sensors (no HIL)!\n");
    }
}

void sensor_mgr_t::update() {
    sensors_prof.start();

    if ( hil_testing_node.getBool("enable") ) {
        // sensors input from sim (via messages)
    } else {
        airdata_mgr.update();
        gps_mgr.update();
        imu_mgr.update();
        power.update();
    }

    if ( hil_testing_node.getBool("enable") and hil_testing_node.getString("inceptors") != "rc" ) {
        // inceptor input from sim (via messages)
    } else {
        inceptors.read();
    }

    sensors_prof.stop();
}

sensor_mgr_t *sensor_mgr = nullptr;