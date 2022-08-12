//
// Created by meng on 2021/2/26.
//

#ifndef GPS_IMU_FUSION_TOOL_H
#define GPS_IMU_FUSION_TOOL_H

#include <eigen3/Eigen/Dense>

//? 坐标系关系
// 右前天->前右地。这里取逆，则为 前右地->右前天。因为数据坐标系是NED，而GPS转换的是右前天，所以需要把IMU相关的数据转换到右前天。
inline void TransformCoordinate(Eigen::Vector3d& vec){
    double kDegree2Radian = M_PI / 180.0;

    Eigen::Quaterniond Q_b_w = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());

    vec = Q_b_w.inverse() * vec;
}

constexpr double kDegree2Radian = M_PI / 180.0;

inline Eigen::Matrix3d BuildSkewMatrix(const Eigen::Vector3d& vec){
    Eigen::Matrix3d matrix;
    matrix << 0.0,     -vec[2],   vec[1],
              vec[2],    0.0,     -vec[0],
              -vec[1],   vec[0],    0.0;

    return matrix;
}


#endif //GPS_IMU_FUSION_TOOL_H
