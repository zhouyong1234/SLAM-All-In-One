/*
 * @Author: your name
 * @Date: 2021-03-26 09:03:40
 * @LastEditTime: 2021-10-07 19:45:02
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /极线可视化/include/FeatureMather.h
 */
#ifndef FEATUREMATHER_H
#define FEATUREMATHER_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>
#include <cassert>
#include "DataPretreat/Config.h"

class FeatureMatcher
{

private:
    cv::Mat Frame_0, Frame_1;
    Config::AppSettings app;

public:
    FeatureMatcher(Config::AppSettings conf, cv::Mat &Img_0, cv::Mat &Img_1);

    void FeatureExtraction(std::vector<cv::DMatch> &matches, std::vector<cv::KeyPoint> &Kp1, std::vector<cv::KeyPoint> &Kp2);

    void OptimizedDec(std::vector<cv::DMatch> &matches, const int &row, double th, double coefficient);

    void SurfExtract( std::vector<cv::DMatch> &matches, std::vector<cv::KeyPoint> &Kp1,                 std::vector<cv::KeyPoint> &Kp2 );

    void SiftExtract( std::vector<cv::DMatch> &matches,
    std::vector<cv::KeyPoint> &Kp1, std::vector<cv::KeyPoint> &Kp2 );

    void OrbExtract( std::vector<cv::DMatch> &matches,
    std::vector<cv::KeyPoint> &Kp1, std::vector<cv::KeyPoint> &Kp2 );
};
#endif // FEATUREMATHER_H
