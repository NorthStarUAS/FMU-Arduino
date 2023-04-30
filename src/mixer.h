#pragma once

#include <math.h>
#if defined(ARDUINO)
# include <eigen.h>
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
using namespace Eigen;

#include "pwm.h"
#include "sensors/sbus/sbus.h"

class mixer_t {
private:
    Matrix<float, PWM_CHANNELS, PWM_CHANNELS> M;
    Matrix<float, PWM_CHANNELS, 1> inputs;

    void sas_update();
    void mixing_update();

public:
    Matrix<float, PWM_CHANNELS, 1> outputs;

    void print_mixer_matrix();
    void setup();
    void sas_defaults();
    void update_matrix(message::config_mixer_t *mix_config );
    void update( float control_norm[SBUS_CHANNELS] );
};

extern mixer_t mixer;
