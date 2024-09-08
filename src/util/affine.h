#pragma once

#include "eigen.h"
#include "Eigen/Geometry"

bool affine_from_points(Eigen::MatrixXf src, Eigen::MatrixXf dst, bool shear, bool scale, Eigen::MatrixXf &M );
void print_matrix(const char *label, Eigen::MatrixXf M);
void print_vector(const char *label, Eigen::VectorXf v);