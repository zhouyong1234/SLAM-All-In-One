#pragma once
#include <iostream>
#include <vector>
#include "Frame/frame.hpp"
#include "camodocal/camera_models/CameraFactory.h"
using namespace camodocal;
class Initializator {
  private:
    const CameraPtr camPtr_;
    std::vector<cv::Point2f> refKeyPoints_,curKeyPoints_;
    Frame* ref_;
    const float focalLength_;
    bool resetReferFlg_;
  public:
    Initializator(CameraPtr camPtr,float focalLength_);
    ~Initializator();
    
    void matchPoints(const Frame* ref,const Frame* cur);
    
    bool setReferenceFrame(Frame* ref);

    void checkParallex(float& parallexAver,float& parallexVar);

    bool runInitialization(const Frame* cur,Eigen::Matrix3d& rotate,Eigen::Vector3d& trans);

    cv::Point2f undistedPts(const cv::Point2f& distPoint);

    bool shouldResetReference() const {
      return resetReferFlg_;
    }

    float checkFundmentalMatrix(cv::Mat F);

    float checkHomography(cv::Mat& H);

};