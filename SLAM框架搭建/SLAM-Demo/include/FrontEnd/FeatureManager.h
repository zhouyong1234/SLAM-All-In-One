/*
 * @Author: Chen Jiahao
 * @Date: 2021-10-08 15:27:55
 * @LastEditors: Chen Jiahao
 * @LastEditTime: 2021-10-08 17:54:48
 * @Description: file content
 * @FilePath: /SLAM-Demo/include/FrontEnd/FeatureManager.h
 */
#ifndef _FEATUREMANAGER_H_
#define _FEATUREMANAGER_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>
#include <cassert>

#include "DataPretreat/Config.h"


class FeatureManager
{
private:
    Config::AppSettings config;

public:
    FeatureManager(Config::AppSettings &config_);
    ~FeatureManager();

    void FeatureExtraction(cv::Mat &imgRGB, std::vector<cv::KeyPoint> &KeyPoints, cv::Mat &Descriptors);

    void FeatureMatch( cv::Mat &Des1, cv::Mat &Des2, std::vector<cv::DMatch> &matches);

    void SurfExtract(cv::Mat &imgRGB, std::vector<cv::KeyPoint> &KeyPoints, cv::Mat &Descriptors);
    void OrbExtract(cv::Mat &imgRGB, std::vector<cv::KeyPoint> &KeyPoints, cv::Mat &Descriptors);
    void SiftExtract(cv::Mat &imgRGB, std::vector<cv::KeyPoint> &KeyPoints, cv::Mat &Descriptors);

    void OptimizedDec(std::vector<cv::DMatch> &matches, const int &row, const double th, const double coefficient);

    void FlannMatch(cv::Mat &Des1, cv::Mat &Des2, std::vector<cv::DMatch> &matches, const double th, const double coefficient);

    void HammingMatch(cv::Mat &Des1, cv::Mat &Des2, std::vector<cv::DMatch> &matches, const double th, const double coefficient);

};

#endif
