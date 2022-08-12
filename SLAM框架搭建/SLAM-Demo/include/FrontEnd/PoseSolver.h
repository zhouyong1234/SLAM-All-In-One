/*
 * @Author: your name
 * @Date: 2021-03-28 17:41:54
 * @LastEditTime: 2021-10-09 14:42:35
 * @LastEditors: Chen Jiahao
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/include/FrontEnd/PoseSolver.h
 */
#ifndef POSESOLVER_H
#define POSESOLVER_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "DataPretreat/Config.h"

class PoseSolver
{
private:
    std::vector<cv::Point3f> PointsInPixelVec_0, PointsInPixelVec_1;
    std::vector<cv::Point3f> PointsInWorldVec_0, PointsInWorldVec_1;
    Config setting;

    Eigen::Matrix4f Pose_0, Pose_1;
    Eigen::Vector3f CamOrientation_0, CamOrientation_1;
    // T12 表示从上一时刻到下一时刻的位移
    Eigen::Matrix4f T12, T12_ransac;

    std::map<int, std::vector<cv::Point3f>* > PixelPointsDict;
    std::map<int, std::vector<cv::Point3f>* > WorldPointsDict;

    Eigen::Matrix3f eigenK;

public:
    PoseSolver(std::vector<cv::Point3f> &coord_0, std::vector<cv::Point3f> &coord_1, Config &config, int initFlag);

    PoseSolver(std::vector<cv::Point3f> &ptInWorld_0, std::vector<cv::Point3f> &ptInPixel_1, Config &config);

    ~PoseSolver();

    void SetOrigin(const int &flag);

    void ComputePnP();

    void KneipPnP(std::vector<int> &idVec, std::vector<cv::Point3f> &PointsInWorldVec_0, std::vector<cv::Point3f> &PointsInPixelVec_1);

    void EPnP_OpenCV(std::vector<int> &idVec, std::vector<cv::Point3f> &PointsInWorldVec_0, std::vector<cv::Point3f> &PointsInPixelVec_1);

    void YuPnP(std::vector<int> &idVec, std::vector<cv::Point3f> &PointsInWorldVec_0,std::vector<cv::Point3f> &PointsInPixelVec_1);

    void solve_quartic_roots(Eigen::Matrix<float, 5, 1> const &factors, std::vector<double> &real_roots);

    double ReProjectError(std::vector<cv::Point3f>&PointsInWorldVec_0,std::vector<cv::Point3f>&PointsInPixelVec_1,const int &i, Eigen::Matrix4f &T);

    double Average_ReProjectError(std::vector<cv::Point3f>&PointsInWorldVec_0
    ,std::vector<cv::Point3f>&PointsInPixelVec_1, Eigen::Matrix4f &T);

    void CheckReProjrctError();

    void Project(Eigen::Vector4f &CamCoord, Eigen::Vector3f & PixelCoord);

    void Unproject(const int &PoseId, const Eigen::Matrix4f &T);

public:
    std::vector<cv::Point3f> GetPointsInWord_0(){ return PointsInWorldVec_0; };

    Eigen::Matrix4f GetT12(){ return T12; };

    Eigen::Matrix4f GetPose();
};

#endif // POSESOLVER_H
