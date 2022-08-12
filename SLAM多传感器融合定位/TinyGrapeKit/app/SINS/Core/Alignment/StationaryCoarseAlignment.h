#pragma once

#include <Eigen/Eigen>

namespace SINS {
    
// return: Rotation from body to navigation frame. C_nb: p_n = C_nb * p_b.
Eigen::Matrix3d StationaryCoarseAlign(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
                                      const Eigen::Vector3d &acc_bias = Eigen::Vector3d::Zero(), 
                                      const Eigen::Vector3d &gyro_bias = Eigen::Vector3d::Zero());

// return: Latitude [radian]                              
double ComputeLatitude(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
                       const Eigen::Vector3d &acc_bias = Eigen::Vector3d::Zero(), 
                       const Eigen::Vector3d &gyro_bias = Eigen::Vector3d::Zero());

}  // namespace SINS