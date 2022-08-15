//
// Created by JinTian on 17/03/2018.
//

#include <opencv/cv.hpp>
#include "visual_odometry.h"

VisualOdometry::VisualOdometry(MonoCamera *camera) {

    this->camera = camera;

    this->featureTracker_ = FeatureTracker();
    this->focal_ = camera->fx();

    this->pp_ = cv::Point2d(camera->cx(), camera->cy());
    this->frameStage_ = STAGE_FIRST_FRAME;
}

VisualOdometry::~VisualOdometry() {
}

// add an image to solving pool
void VisualOdometry::addImage(const cv::Mat &img, int frameID) {

    // std::cout << img.empty() << std::endl;
    // std::cout << img.type() << std::endl;
    // std::cout << img.cols << std::endl;
    // std::cout << camera->width() << std::endl;


    if (img.empty() || img.type() != CV_8UC1 || img.cols != camera->width()) {
        throw std::runtime_error("Frame: image not same size as camera.");
    }

    // std::cout << frameStage_ << std::endl;

    newFrame_ = img;
    bool res = true;
    if (frameStage_ == STAGE_DEFAULT_FRAME) {
        res = processFrame(frameID);
    }else if(frameStage_ == STAGE_SECOND_FRAME) {
        res = processSecondFrame();
    }else if(frameStage_ == STAGE_FIRST_FRAME) {
        res = processFirstFrame();
    }
    lastFrame_ = newFrame_;
}

bool VisualOdometry::processFirstFrame() {
    // calculate the reference feature of new frame
    this->featureTracker_.detect(newFrame_, pxRef_);
    frameStage_ = STAGE_SECOND_FRAME;
    return true;
}

bool VisualOdometry::processSecondFrame() {
    // using KFT track second frame feature
    this->featureTracker_.track(lastFrame_, newFrame_, pxRef_, pxCur_, disparities_);

    // E: Essential Matrix
    // R: rotation matrix
    // t: transfer matrix
    cv::Mat E, R, t, mask;
    E = cv::findEssentialMat(pxCur_, pxRef_, focal_, pp_, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, pxCur_, pxRef_, R, t, focal_, pp_, mask);
    curR_ = R.clone();
    curT_ = t.clone();
    frameStage_ = STAGE_DEFAULT_FRAME;
    pxRef_ = pxCur_;
    return true;
}

bool VisualOdometry::processFrame(int frameID) {
    double scale = 0.15;
    this->featureTracker_.track(lastFrame_, newFrame_, pxRef_, pxCur_, disparities_);
    cv::Mat E, R, t, mask;
    E = cv::findEssentialMat(pxCur_, pxRef_, focal_, pp_, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, pxCur_, pxRef_, R, t, focal_, pp_, mask);

    // std::cout << "t: " << t.t() << std::endl;

    scale = getAbsoluteScale(frameID);
    // if scale less than 0.1, R and t maybe wrong, keep last value
    if (scale >= 0.15) {
        curT_ = curT_ + scale*(curR_*t);
        curR_ = R*curR_;
    }
    if (pxRef_.size() < kMinNumFeature) {
        // std::cout << "detect track" << std::endl;
        this->featureTracker_.detect(newFrame_, pxRef_);
        this->featureTracker_.track(lastFrame_, newFrame_, pxRef_, pxCur_, disparities_);

    }
    pxRef_ = pxCur_;
    return true;
}

double VisualOdometry::getAbsoluteScale(int frameID) {
    return 0.15;
}