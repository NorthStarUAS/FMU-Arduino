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
#if defined(ARDUINO)
# include <eigen.h>
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
using namespace Eigen;

#include "../nav_common/structs.h"

// define some types for notational convenience and consistency
typedef Matrix<float,9,9>   Matrix9f;
typedef Matrix<float,12,12> Matrix12f;
typedef Matrix<float,15,15> Matrix15f;
typedef Matrix<float,9,15>  Matrix9x15f;
typedef Matrix<float,15,9>  Matrix15x9f;
typedef Matrix<float,15,12> Matrix15x12f;
typedef Matrix<float,9,1>   Vector9f;
typedef Matrix<float,15,1>  Vector15f;

class EKF15_mag {

public:

    EKF15_mag() {
	default_config();
    }
    ~EKF15_mag() {}

    // set/get error characteristics of navigation sensors
    void set_config(NAVconfig config);
    NAVconfig get_config();
    void default_config();

    // main interface
    void init(IMUdata imu, GPSdata gps);
    void set_ideal_mag_vector_ned(Vector3f v);
    void time_update(IMUdata imu);
    void measurement_update(IMUdata imu, GPSdata gps);

    NAVdata get_nav();

private:

    Matrix15f F, PHI, P, Qw, Q, ImKH, KRKt, I15 /* identity */;
    Matrix15x12f G;
    Matrix15x9f K;
    Vector15f x;
    Matrix12f Rw;
    Matrix9x15f H;
    Matrix9f R;
    Vector9f y;
    Matrix3f C_N2B, C_B2N, I3 /* identity */, temp33;
    Vector3d pos_ins_ecef, pos_gps, pos_gps_ecef;
    Vector3f grav, f_b, om_ib, pos_ins_ned, pos_gps_ned, dx, mag_ned;

    Quaternionf quat;
    float tprev;

    IMUdata imu_last;
    NAVconfig config;
    NAVdata nav;
};
