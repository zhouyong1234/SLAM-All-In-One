#pragma once

#include <cmath>

#include <Eigen/Eigen>

#include "Utils.h"

namespace SINS {

inline Eigen::Matrix3d SO3Exp(const Eigen::Vector3d &v) {
    double theta = v.norm();
    if (theta < 1e-14) {
        return Eigen::Matrix3d::Identity() + SkewMat(v);
    }

    return Eigen::AngleAxisd(theta, v.normalized()).toRotationMatrix();
} 

inline Eigen::Matrix3d NormalizeRotMat(const Eigen::Matrix3d &rot) {
    Eigen::Quaterniond quat(rot);
    quat.normalize();
    return quat.toRotationMatrix();
}

inline Eigen::Quaterniond QuatExp(const Eigen::Vector3d &v) {
    const double half_theta = 0.5 * v.norm();
    if (half_theta < 1e-14) {
        const Eigen::Vector3d im_part = 0.5 * v;
        return Eigen::Quaterniond(1.0, im_part.x(), im_part.y(), im_part.z()).normalized(); 
    }

    const Eigen::Vector3d im_part = v.normalized() * std::sin(half_theta);
    return Eigen::Quaterniond(std::cos(half_theta), im_part.x(), im_part.y(), im_part.z()).normalized();
}

inline Eigen::Vector3d MatToAtt(const Eigen::Matrix3d &rot_mat) {
    return Eigen::Vector3d(
        std::asin(rot_mat(2, 1)), 
        std::atan2(-rot_mat(2, 0), rot_mat(2, 2)), 
        std::atan2(-rot_mat(0, 1), rot_mat(1, 1)));
}

inline Eigen::Matrix3d AttToMat(const Eigen::Vector3d &att) {
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(att.z(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(att.x(), Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(att.y(), Eigen::Vector3d::UnitY());
    return rot;
}

inline Eigen::Quaterniond AttToQuat(const Eigen::Vector3d &att) {
    return Eigen::Quaterniond(AttToMat(att));
}

inline Eigen::Vector3d QuatToAtt(const Eigen::Quaterniond &quat) {
    return MatToAtt(quat.toRotationMatrix());
}

} // namespace SINS