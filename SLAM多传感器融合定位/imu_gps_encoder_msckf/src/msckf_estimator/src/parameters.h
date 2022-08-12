#ifndef PARAMETER_H
#define PARAMETER_H

#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
// #include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>

const double FOCAL_LENGTH = (816.90378992770002 + 811.56803828490001) / 2;

extern std::string IMU_TOPIC;
extern std::string GPS_TOPIC;
extern std::string ENCODER_TOPIC;
extern Eigen::Vector3d t_o_g;
extern Eigen::Matrix3d R_o_i;
extern Eigen::Vector3d t_o_i;
extern Eigen::Matrix3d R_o_c;
extern Eigen::Vector3d t_o_c;
extern double acc_noise;
extern double acc_bias_noise;
extern double gyro_noise;
extern double gyro_bias_noise;
extern double left_wheel_diameter;
extern double right_wheel_diameter;
extern double wheel_base;

void readParameters(ros::NodeHandle &n);

inline Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w <<  0.,   -v(2),  v(1),
		v(2),  0.,   -v(0),
		-v(1),  v(0),  0.;
    return w;
}

#endif