#pragma once

#include <Eigen/Eigen>

namespace SINS {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

inline Eigen::Matrix3d SkewMat(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skew_mat;

    skew_mat << 0.0, -v.z(), v.y(),
                v.z(), 0.0, -v.x(),
                -v.y(), v.x(), 0.0;

    return skew_mat;
}

inline double sec(double theta) {
    return 1. / std::cos(theta);
}

}  // namespace SINS