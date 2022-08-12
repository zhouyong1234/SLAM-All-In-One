/*
 * @Author: Chen Jiahao
 * @Date: 2021-10-08 15:27:44
 * @LastEditors: Chen Jiahao
 * @LastEditTime: 2021-10-08 17:56:01
 * @Description: file content
 * @FilePath: /SLAM-Demo/src/FrontEnd/FeatureManager.cc
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cassert>
#include <mutex>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "FrontEnd/FeatureManager.h"

// #define DEBUG
#ifdef DEBUG
#define CHECK_INFO(x) std::cout << "[DEBUG] " << x << std::endl;
#define CHECK_INFO_2(x, y) std::cout << "[DEBUG] " << x << y << std::endl;
#else
#define CHECK_INFO(x)      //std::cout << x << std::endl;
#define CHECK_INFO_2(x, y) //std::cout << "[DEBUG] " << x << y << std::endl;
#endif

FeatureManager::FeatureManager(Config::AppSettings &config_) : config(config_)
{
}

FeatureManager::~FeatureManager()
{
}

void FeatureManager::FeatureExtraction(
    cv::Mat &imgRGB, std::vector<cv::KeyPoint> &KeyPoints, cv::Mat &Descriptors)
{
    if (config.ORB_Features == true){
        OrbExtract(imgRGB, KeyPoints, Descriptors);
    }else if (config.SIFT_Features == true){
        SiftExtract(imgRGB, KeyPoints, Descriptors);
    }else if (config.SURF_Features == true){
        SurfExtract(imgRGB, KeyPoints, Descriptors);
    }else{
        std::cerr << "[ERRO] NO Extraction Method" << std::endl;
    }

}

void FeatureManager::FeatureMatch(
    cv::Mat &Des1, cv::Mat &Des2, std::vector<cv::DMatch> &matches)
{
    if (config.ORB_Features == true){
        HammingMatch(Des1, Des2, matches, 30.0, 2);
    }else if (config.SIFT_Features == true){
        FlannMatch( Des1, Des2, matches, 100, 2);
    }else if (config.SURF_Features == true){
        FlannMatch(Des1, Des2, matches, 0.01, 3);
    }else{
        std::cerr << "[ERRO] NO Extraction Method" << std::endl;
    }
}

void FeatureManager::SurfExtract(
    cv::Mat &imgRGB, std::vector<cv::KeyPoint> &KeyPoints, cv::Mat &Descriptors)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SURF::create(500);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::xfeatures2d::SURF::create();
    detector->detect(imgRGB, KeyPoints);
    descriptor->compute(imgRGB, KeyPoints, Descriptors);

}

void FeatureManager::SiftExtract(cv::Mat &imgRGB, std::vector<cv::KeyPoint> &KeyPoints, cv::Mat &Descriptors)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SIFT::create(500);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::xfeatures2d::SIFT::create();
    detector->detect(imgRGB, KeyPoints);
    descriptor->compute(imgRGB, KeyPoints, Descriptors);
    // OptimizedDec(matches, Des1.rows, 100, 2);

}

void FeatureManager::OrbExtract(cv::Mat &imgRGB, std::vector<cv::KeyPoint> &KeyPoints, cv::Mat &Descriptors)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    detector->detect(imgRGB, KeyPoints);
    descriptor->compute(imgRGB, KeyPoints, Descriptors);

    // OptimizedDec(matches, Des1.rows, 30.0, 2);
}

//sift，surf，为CV32F直接套用汉明匹配则报错
void FeatureManager::FlannMatch(
    cv::Mat &Des1, cv::Mat &Des2,
    std::vector<cv::DMatch> &matches,
    const double th, const double coefficient)
{
    cv::FlannBasedMatcher matcher;
    matcher.match(Des1, Des2, matches);
    CHECK_INFO_2("优化前匹配数量： ", matches.size());

    OptimizedDec(matches, Des1.rows, th, coefficient);
    CHECK_INFO_2("优化后匹配数量： ", matches.size());
}

void FeatureManager::HammingMatch(
    cv::Mat &Des1, cv::Mat &Des2,
    std::vector<cv::DMatch> &matches,
    const double th, const double coefficient)
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-HammingLUT");
    matcher->match(Des1, Des2, matches);
    CHECK_INFO_2("优化前匹配数量： ", matches.size());
    // ORB: 30.0, 2
    OptimizedDec(matches, Des1.rows, th, coefficient);
    CHECK_INFO_2("优化后匹配数量： ", matches.size());
}

void FeatureManager::OptimizedDec(std::vector<cv::DMatch> &matches, const int &row, const double th, const double coefficient)
{
    std::vector<cv::DMatch> Gmatches;
    double min_dist = 10000, max_dist = 0;
    for (int i(0); i < row; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }
    //cout << "max :" << max_dist << endl << "min :" << min_dist << endl;
    for (int i = 0; i < row; i++)
    {
        if (matches[i].distance <= std::max(coefficient * min_dist, th))
        {
            Gmatches.push_back(matches[i]);
        }
    }
    matches = Gmatches;
}
