/*
 * @Author: your name
 * @Date: 2021-03-26 14:55:09
 * @LastEditTime: 2021-10-08 14:04:52
 * @LastEditors: Chen Jiahao
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/include/DataPretreat/DepthMap.h
 */
#ifndef DEPTHMAP_H
#define DEPTHMAP_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include "DataPretreat/Config.h"

class DepthMap
{
private:
    cv::Mat imgRGBL, imgRGBR;
    cv::Mat imgDepth;
public:
    DepthMap();

    void RGBDDepthBuilder(cv::Mat &imgDepth_, Config &config);

    void StereoDepthBuilder(cv::Mat &imgRGBL_, cv::Mat &imgRGBR_, Config &config);

    void insertDepth32f(cv::Mat& depth);

    void CheckDepthMap(cv::Mat& depth, cv::Mat &depth_show);

    cv::Mat SGBM(Config &config, cv::Mat &img_left, cv::Mat &img_right);

    cv::Mat GetDepthMap();

    double GetDepth(cv::Point2f &coordinate);

};

void Mouse_Callback(int event,int x,int y,int flags,void *param);



#endif //DEPTHMAP_H
