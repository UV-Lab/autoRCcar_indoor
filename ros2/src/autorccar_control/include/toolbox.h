#ifndef INSTOOLBOX_H
#define INSTOOLBOX_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define _USE_MATH_DEFINES
#include <cmath>

Eigen::Matrix<double, 3, 3> Quat2DCM(const Eigen::Quaterniond &quat);
Eigen::Vector3d Quat2Euler(const Eigen::Quaterniond &quat);
Eigen::Quaterniond Euler2Quat(const Eigen::Vector3d &eulr);
Eigen::Quaterniond QuatProduct(const Eigen::Quaterniond &quat1, const Eigen::Quaterniond &quat2);

Eigen::Vector3d ECEF2LLH(const Eigen::Vector3d & pos);

#endif