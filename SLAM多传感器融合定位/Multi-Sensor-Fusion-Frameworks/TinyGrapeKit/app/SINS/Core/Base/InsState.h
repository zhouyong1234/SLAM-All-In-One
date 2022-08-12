#pragma once

#include <Eigen/Eigen>

namespace SINS {

struct InsState {
    double time;  // [s]

    // AVP
    Eigen::Quaterniond orientation;  // q_nb
    Eigen::Vector3d velocity;        // v_nb
    Eigen::Vector3d lat_lon_hei;     // [rad, rad, m] 

    // IMU bias.
    Eigen::Vector3d gyro_bias;   
    Eigen::Vector3d acc_bias;

    // IMU Reading.
    Eigen::Vector3d ub_acc;  // [m/s^2]
    Eigen::Vector3d ub_gyro; // [rad / s]

    // Earth params.
    bool update_earth = false;
    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.8);
    double Rm;
    double Rn;
    Eigen::Vector3d Wnie; 
    Eigen::Vector3d Wnen; 
    Eigen::Vector3d Wnin; 

    // Mid use
    Eigen::Matrix3d Mpv;
};

}  // namespace SINS