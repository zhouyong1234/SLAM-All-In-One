#include "StationaryCoarseAlignment.h"

namespace SINS {

// Ref：严恭敏. 捷联惯导算法与组合导航原理. p196~p197.
Eigen::Matrix3d StationaryCoarseAlign(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
                                      const Eigen::Vector3d &acc_bias, const Eigen::Vector3d &gyro_bias) {
    // Remove bias.
    const Eigen::Vector3d unbias_acc = acc - acc_bias;
    const Eigen::Vector3d ubias_gyro = gyro - gyro_bias;

    // Eq 7.1.23. 
    const Eigen::Vector3d row_1 = unbias_acc.cross(ubias_gyro);
    const Eigen::Vector3d row_2 = unbias_acc.cross(ubias_gyro).cross(unbias_acc);
    const Eigen::Vector3d row_3 = unbias_acc;

    Eigen::Matrix3d C_nb;
    C_nb.block<1, 3>(0, 0) = -row_1.transpose() / row_1.norm();
    C_nb.block<1, 3>(1, 0) = row_2.transpose() / row_2.norm();
    C_nb.block<1, 3>(2, 0) = row_3.transpose() / row_3.norm();

    return C_nb;
}
                                      
double ComputeLatitude(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
                       const Eigen::Vector3d &acc_bias, const Eigen::Vector3d &gyro_bias) {
    const Eigen::Vector3d unbias_acc = acc - acc_bias;
    const Eigen::Vector3d ubias_gyro = gyro - gyro_bias;
    const double acc_gyro_angle = std::acos((unbias_acc.transpose() * ubias_gyro).value() / (unbias_acc.norm() * ubias_gyro.norm()));

    return M_PI_2 - acc_gyro_angle;
}

}  // namespace SINS