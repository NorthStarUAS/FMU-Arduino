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
}

void sensor_mgr_t::update() {
    sensors_prof.start();

    if ( not status_node.getBool("HIL_mode") ) {
        airdata_mgr.update();
        gps_mgr.update();
        imu_mgr.update();
        power.update();
        inceptors.read();
    }

    sensors_prof.stop();
}

sensor_mgr_t *sensor_mgr = nullptr;