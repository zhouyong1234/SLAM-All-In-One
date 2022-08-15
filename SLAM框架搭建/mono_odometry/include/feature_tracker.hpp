#ifndef FEATURE_TRACKER_H
#define FEATURE_TRACKER_H

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"

class FeatureTracker{
public:
    void track(cv::Mat imageRef, cv::Mat imageCur,
    std::vector<cv::Point2f> &pxRef, std::vector<cv::Point2f> &pxCur,
    std::vector<double> &disparities);

    void detect(cv::Mat image, std::vector<cv::Point2f>& pxVec);
};

#endif