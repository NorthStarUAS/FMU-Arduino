#pragma once

#include <math.h>
#if defined(ARDUINO)
# include <Eigen.h>
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
using namespace Eigen;

#include "aura4_messages.h"
#include "pwm.h"
#include "sensors/sbus/sbus.h"

class mixer_t {
private:
    Matrix<float, PWM_CHANNELS, PWM_CHANNELS> M;
    Matrix<float, PWM_CHANNELS, 1> inputs;
    
    void sas_update();
    void mixing_update();

public:
    //message::config_actuators_t config;
    // float outputs[PWM_CHANNELS]; // mixed output values (normalized)
    Matrix<float, PWM_CHANNELS, 1> outputs;
    
    void setup();
    void act_gain_defaults();
    void sas_defaults();
    void mixing_defaults();
    void update( float control_norm[SBUS_CHANNELS] );
};

extern mixer_t mixer;
