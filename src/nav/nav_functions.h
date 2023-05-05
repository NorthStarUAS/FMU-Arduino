/*! \file nav_functions.h
 *	\brief Auxiliary functions for nav filter header file
 *
 *	\details
 *     Module:          navfunc.h
 *     Modified:        Brian Taylor (convert to eigen3)
 *						Gokhan Inalhan (remaining)
 *                      Demoz Gebre (first three functions)
 *                      Adhika Lie
 *                      Jung Soon Jang
 *     Description:     navfunc.h contains all the variable,
 *                      constants and function prototypes that are
 *                      used with the inertial navigation software.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: nav_functions.h 922 2012-10-17 19:14:09Z joh07594 $
 */


#pragma once

// #include "setup_board.h"

#include <math.h>
#include "eigen.h"
// #include "eigen3/Eigen/Core"
// #include "eigen3/Eigen/Geometry"

// Constants
const double EarthRadius = 6378137.0;        // earth semi-major axis radius (m)
const double ECC2 = 0.0066943799901;         // major eccentricity squared

// Constants that are no longer used
// const double EarthRate = 0.00007292115;      // rotation rate of earth (rad/sec)
// const double Eccentricity = 0.0818191908426; // major eccentricity of earth ellipsoid
// const double Flattening = 0.0033528106650;   // flattening of the ellipsoid
// const double Gravity0 = 9.7803730;           // zeroth coefficient for gravity model
// const double Gravity1 = 0.0052891;           // first coefficient for the gravity model
// const double Gravity2 = 0.0000059;           // second coefficient for the gravity model
// const double GravityNom = 9.81;              // nominal gravity
// const double Schuler2 = 1.533421593170545E-06; // Schuler Frequency (rad/sec) Squared

// This function calculates the rate of change of latitude, longitude,
// and altitude using WGS-84.
Eigen::Vector3f llarate(Eigen::Vector3f V, Eigen::Vector3d lla);

// This function calculates the angular velocity of the NED frame,
// also known as the navigation rate using WGS-84.
Eigen::Vector3d navrate(Eigen::Vector3d V, Eigen::Vector3d lla);

// This function calculates the ECEF Coordinate given the
// Latitude, Longitude and Altitude.
Eigen::Vector3d lla2ecef(Eigen::Vector3d lla);

// This function calculates the Latitude, Longitude and Altitude given
// the ECEF Coordinates.
Eigen::Vector3d ecef2lla( Eigen::Vector3d ecef_pos );

// This function converts a vector in ecef to ned coordinate centered
// at pos_ref.
Eigen::Vector3f ecef2ned(Eigen::Vector3d ecef, Eigen::Vector3d pos_ref);

// Return a quaternion rotation from the earth centered to the
// simulation usual horizontal local frame from given longitude and
// latitude.  The horizontal local frame used in simulations is the
// frame with x-axis pointing north, the y-axis pointing eastwards and
// the z axis pointing downwards.  (Returns the ecef2ned
// transformation as a quaternion.)
Eigen::Quaterniond lla2quat(double lon_rad, double lat_rad);

// This function gives a skew symmetric matrix from a given vector w
Eigen::Matrix3f sk(Eigen::Vector3f w);

// Quaternion to euler angle: returns phi, the, psi as a vector
Eigen::Vector3f quat2eul(Eigen::Quaternionf q);

// Computes a quaternion from the given euler angles
Eigen::Quaternionf eul2quat(float phi, float the, float psi);

// Quaternion to C_N2B
Eigen::Matrix3f quat2dcm(Eigen::Quaternionf q);
