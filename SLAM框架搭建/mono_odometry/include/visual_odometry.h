//
// Created by JinTian on 17/03/2018.
//

#ifndef MONO_ODOMETRY_VISUAL_ODOMETRY_H
#define MONO_ODOMETRY_VISUAL_ODOMETRY_H

#include "feature_tracker.hpp"
#include "mono_camera.h"

class VisualOdometry{

public:
    enum FrameStage{
        STAGE_FIRST_FRAME,
        STAGE_SECOND_FRAME,
        STAGE_DEFAULT_FRAME
    };

    VisualOdometry(MonoCamera *camera);
    ~VisualOdometry();

    void addImage(const cv::Mat& img, int frameID);

    // get current rotation
    inline cv::Mat getCurrentR() {return curR_;}
    // get current transfer
    inline cv::Mat getCurrentT() {return curT_;}

    int kMinNumFeature = 2000;
    inline void setMinNumFeature(int minNumFeature) {kMinNumFeature = minNumFeature;};




protected:
    virtual bool processFirstFrame();
    virtual bool processSecondFrame();
    virtual bool processFrame(int frameID);
    double getAbsoluteScale(int frameID);

protected:
    MonoCamera *camera;

    FrameStage frameStage_;
    FeatureTracker featureTracker_;

    // new frame
    cv::Mat newFrame_;
    // last frame
    cv::Mat lastFrame_;

    cv::Mat curR_;
    // current panning
    cv::Mat curT_;

    std::vector<cv::Point2f> pxRef_;
    std::vector<cv::Point2f> pxCur_;

    std::vector<double> disparities_;

    // we have to know camera focal and center point
    double focal_;
    cv::Point2d pp_;

};


#endif //MONO_ODOMETRY_VISUAL_ODOMETRY_H
