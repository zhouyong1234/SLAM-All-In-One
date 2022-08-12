#include "Frame/frame.hpp"

Frame::Frame() {

}

Frame::Frame(double t,bool keyframe,std::map<int,cv::Point2f>& keypoints)
:t_(t),keyframe_(keyframe) {
  keypoints_ = keypoints;
}

Frame::~Frame() {
  keypoints_.clear();
}

