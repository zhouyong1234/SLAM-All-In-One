/*
 * @Author: Chen Jiahao
 * @Date: 2021-10-08 11:33:08
 * @LastEditors: Chen Jiahao
 * @LastEditTime: 2021-10-09 15:12:26
 * @Description: file content
 * @FilePath: /SLAM-Demo/include/DataPretreat/Frame.h
 */

#ifndef _FRAME_H_
#define _FRAME_H_

#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "DataPretreat/Config.h"
#include "DataPretreat/DepthMap.h"
#include "FrontEnd/FeatureManager.h"

class Frame
{
private:
    cv::Mat imgRGBL, imgRGBR, imgDepth;
    cv::Mat imgResizeL, imgResizeR;
    cv::Mat imgGrayL, imgGrayR;

    std::vector<cv::KeyPoint> KeyPoints;
    cv::Mat Descriptors;
public:
    double timeStamps;

public:
    Frame();
    ~Frame();

    Frame(const cv::Mat &imgRGBL_, const cv::Mat &imageR_, Config &config_, std::string &timeStamps_);

    void BuildDepthMap(const cv::Mat &imageR_, Config &config_);

    void GetKPDes(std::vector<cv::KeyPoint> &KeyPoints_, cv::Mat &Descriptors_);

    cv::Mat GetDepthMap();

    cv::Mat GetIMGLeft();
};


#endif
