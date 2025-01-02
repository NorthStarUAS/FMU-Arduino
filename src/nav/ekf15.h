/*! \file EKF_15state.c
 *	\brief 15 state EKF navigation filter
 *
 *	\details  15 state EKF navigation filter using loosely integrated INS/GPS architecture.
 * 	Time update is done after every IMU data acquisition and GPS measurement
 * 	update is done every time the new data flag in the GPS data packet is set. Designed by Adhika Lie.
 *	Attitude is parameterized using quaternions.
 *	Estimates IMU bias errors.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 */

#pragma once

#if defined(ARDUINO)
# include <eigen.h>
# include <Eigen/Geometry>
#else
# include <math.h>
# include <eigen3/Eigen/Core>
# include <eigen3/Eigen/Geometry>
# include <eigen3/Eigen/LU>
#endif

using namespace Eigen;

#include "nav_structs.h"

// define some types for notational convenience and consistency
typedef Matrix<float,6,1> Vector6f;
typedef Matrix<float,15,1> Vector15f;

class EKF15 {

public:

    EKF15() {
        default_config();
    }
    ~EKF15() {}

    // set/get error characteristics of navigation sensors
    void set_config(NAVconfig _config);
    NAVconfig get_config();
    void default_config();

    // main interface
    void init(IMUdata imu, GPSdata gps);
    void time_update(IMUdata imu);
    void measurement_update(GPSdata gps);

    NAVdata get_nav();

private:

    // make our big matrices dynamic (so they get allocated on the
    // heap) to play nice on embedded systems with small stacks.
    MatrixXf F, PHI, P, Qw, Q, ImKH, KRKt, I15; // 15x15
    MatrixXf G;                                 // 15x12
    MatrixXf K;                                 // 15x6
    MatrixXf Rw;                                // 12x12
    MatrixXf H;                                 // 6x15
    MatrixXf R;                                 // 6x6
    Vector15f x;                                // 15x1
    Vector6f y;                                 // 6x1
    Matrix3f C_N2B, C_B2N, I3, temp33;
    Vector3f grav, f_b, om_ib, dx, mag_ned;

    Quaternionf quat;

    IMUdata imu_last;
    NAVconfig config;
    NAVdata nav;
};
