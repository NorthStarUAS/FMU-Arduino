// IMU wrapper class

#include "../../aura3_messages.h"

class imu_t {
 private:
    float p_calib = 0.0;
    float q_calib = 0.0;
    float r_calib = 0.0;
    
    void rotate(float v0, float v1, float v2,
                float *r0, float *r1, float *r2,
                message::config_imu_t &config_imu);
    void calibrate_gyros(float gx, float gy, float gz);

 public:
    // 0 = uncalibrated, 1 = calibration in progress, 2 = calibration finished
    int gyros_calibrated = 0;
    
    unsigned long imu_micros = 0;
    float ax = 0.0;
    float ay = 0.0;
    float az = 0.0;
    float p = 0.0;
    float q = 0.0;
    float r = 0.0;
    float hx = 0.0;
    float hy = 0.0;
    float hz = 0.0;
    float temp = 0.0;
    
    void defaults(message::config_imu_t &config_imu);
    void setup(message::config_imu_t &config_imu);
    void update(message::config_imu_t &config_imu);
};
