/*
 * @Author: Chen Jiahao
 * @Date: 2021-10-08 14:09:58
 * @LastEditors: Chen Jiahao
 * @LastEditTime: 2021-10-10 09:13:19
 * @Description: file content
 * @FilePath: /SLAM-Demo/include/FrontEnd/Tracking.h
 */
#ifndef _TRACKING_H_
#define _TRACKING_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "DataPretreat/Config.h"
#include "DataPretreat/Frame.h"
#include "FrontEnd/PoseSolver.h"

class Tracking
{
private:
    Eigen::Matrix4f initPose;
    Eigen::Matrix4f currPose;

    std::string saveMode = "TUM";
    struct
    {
        size_t N = 0;
        std::deque<double> timeDeque;
        std::deque<Eigen::Matrix4f> poseDeque;
    } trajectory;

public:
    Tracking(Eigen::Matrix4f &initPose_, Config &conf);

    ~Tracking();

    void RunTracking(Frame &preFrame, Frame &curFrame, Config &conf);

    void FusionPosAndDepth(const cv::Mat &preDepth, const cv::Mat &curDepth,
        const std::vector<cv::KeyPoint> &KPs1, const std::vector<cv::KeyPoint> &KPs2,
        const cv::Mat &Des1, const cv::Mat &Des2,
        const std::vector<cv::DMatch> matches,
        std::vector<cv::Point3f> &prePixelPoints, std::vector<cv::Point3f> &curPixelPoints);

    void RecordPose(Eigen::Matrix4f &T, double &timeStapms);

    void SavePose(std::string &filePath);
};


#endif
