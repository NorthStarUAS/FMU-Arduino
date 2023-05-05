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

#include <math.h>
#include "eigen.h"
// #include "eigen3/Eigen/Core"
// #include "eigen3/Eigen/Geometry"
// #include "eigen3/Eigen/LU"

#include "nav_structs.h"

// define some types for notational convenience and consistency
//typedef Eigen::Matrix<float,9,9>   Matrix9f;
//typedef Eigen::Matrix<float,12,12> Matrix12f;
//typedef Eigen::Matrix<float,15,15> Matrix15f;
//typedef Eigen::Matrix<float,9,15>  Matrix9x15f;
//typedef Eigen::Matrix<float,15,9>  Matrix15x9f;
//typedef Eigen::Matrix<float,15,12> Matrix15x12f;
typedef Eigen::Matrix<float,9,1> Vector9f;
typedef Eigen::Matrix<float,15,1> Vector15f;

class EKF15_mag {

public:

    EKF15_mag() {
	default_config();
    }
    ~EKF15_mag() {}

    // set/get error characteristics of navigation sensors
    void set_config(NAVconfig _config);
    NAVconfig get_config();
    void default_config();

    // main interface
    void init(IMUdata imu, GPSdata gps);
    void set_ideal_mag_vector_ned(Eigen::Vector3f v);
    void time_update(IMUdata imu);
    void measurement_update(IMUdata imu, GPSdata gps);

    NAVdata get_nav();

private:

    // make our big matrices dynamic (so they get allocated on the
    // heap) to play nice on embedded systems with small stacks.
    Eigen::MatrixXf F, PHI, P, Qw, Q, ImKH, KRKt, I15; // 15x15
    Eigen::MatrixXf G;                                 // 15x12
    Eigen::MatrixXf K;                                 // 15x9
    Eigen::MatrixXf Rw;                                // 12x12
    Eigen::MatrixXf H;                                 // 9x15
    Eigen::MatrixXf R;                                 // 6x6
    Vector15f x;                                       // 15x11
    Vector9f y;                                        // 9x1
    Eigen::Matrix3f C_N2B, C_B2N, I3 /* identity */, temp33;
    Eigen::Vector3f grav, f_b, om_ib, pos_ins_ned, pos_gps_ned, dx, mag_ned;

    Eigen::Quaternionf quat;

    IMUdata imu_last;
    NAVconfig config;
    NAVdata nav;
};
