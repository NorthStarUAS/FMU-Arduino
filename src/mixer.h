#pragma once

#include <Arduino.h>

#include <math.h>
#if defined(ARDUINO)
# include <eigen.h>
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
using namespace Eigen;

#include "props2.h"

class mixer_t {

private:
    Eigen::MatrixXf M;
    Eigen::VectorXf inputs, outputs;

    void sas_update();
    void mixing_update();

    PropertyNode effectors_node;
    PropertyNode imu_node;
    PropertyNode pilot_node;
    PropertyNode stab_roll_node;
    PropertyNode stab_pitch_node;
    PropertyNode stab_yaw_node;
    PropertyNode stab_tune_node;
    PropertyNode switches_node;

public:

    void print_mixer_matrix();
    void init();
    void sas_defaults();
    void update_matrix();
    void update();
};
