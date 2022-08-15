/*
 * Common.hpp
 *
 *  Created on: Feb 9, 2014
 *      Author: Bloeschm
 */

#ifndef LWF_COMMON_HPP_
#define LWF_COMMON_HPP_

#include <map>
#include <type_traits>
#include <tuple>
#include <Eigen/Dense>
#include <iostream>
#include "kindr/Core"
#include "lightweight_filtering/PropertyHandler.hpp"

typedef kindr::RotationQuaternionPD QPD;
typedef kindr::RotationMatrixPD MPD;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::VectorXd VXD;
typedef Eigen::MatrixXd MXD;
inline M3D gSM(const V3D& vec){
  return kindr::getSkewMatrixFromVector(vec);
}

static void enforceSymmetry(MXD& mat){
  mat = 0.5*(mat+mat.transpose()).eval();
}

inline M3D Lmat (const V3D& a) {
  return kindr::getJacobianOfExponentialMap(a);
}

namespace LWF{
  enum FilteringMode{
    ModeEKF,
    ModeUKF,
    ModeIEKF
  };
}

#endif /* LWF_COMMON_HPP_ */
