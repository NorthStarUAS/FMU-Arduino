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
    delay(100);

    // power sensing
    analogReadResolution(16);   // set up ADC0
    power.init();

    // initialize the pilot interface (RC in, out & mixer)
    inceptors.init();
}

void sensor_mgr_t::update() {
    // poll the pressure sensors
    airdata_mgr.update();

    // 2. Check for gps updates
    gps_mgr.update();

    // 1. Sense motion (top priority, used for timing sync downstream.)
    imu_mgr.update();

    if ( !status_node.getBool("HIL_mode") ) {
        // read power values
        power.update();
    }

    inceptors.read();
}

sensor_mgr_t *sensor_mgr;