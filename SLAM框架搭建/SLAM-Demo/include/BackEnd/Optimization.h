/*
 * @Author: your name
 * @Date: 2021-04-02 10:37:55
 * @LastEditTime: 2021-10-07 19:44:47
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/include/Optimization.h
 */

#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "DataPretreat/Config.h"

class Optimization
{
private:
    Config setting;
public:
    Optimization(Config &config);
    ~Optimization();

    // template<T, N>
    // void LeastSquare(std::vector<T> &Observation, N &Result);

    void Kneip_Ransac(std::vector<cv::Point3f> &PointsInWorldVec_0, std::vector<cv::Point3f> &PointsInPixelVec_1, std::vector<int> &Interior, Eigen::Matrix4f &Pose);

    void BA_OptimizePose(std::vector<cv::Point3f> &points_3d ,std::vector<cv::Point3f> &features, Eigen::Matrix4f &Pose);
};


#endif //OPTIMIZATION_H

