#pragma once

#include <opencv2/opencv.hpp>
#include "NanoStation.hpp"
namespace nnstation {

struct TimedCvMat {
  double t;
  cv::Mat img;
};

class BottomClient : public NanoClient<TimedCvMat> {
 public:
  typedef NanoClient<TimedCvMat> Base;
  typedef Base::mtParsed mtParsed;
  using Base::connect;
  using Base::msgLen;
  using Base::startRecv;
  using Base::subscribe;
  using Base::getOldestReplayTime;

  uint32_t width_;
  uint32_t height_;

  BottomClient() : width_(0), height_(0), imgSize_(0), bufSize_(0) {};

  ~BottomClient() override = default;

  void setImageSize(uint32_t width, uint32_t height) {
    width_ = width;
    height_ = height;
    imgSize_ = width_ * height_;
    bufSize_ = imgSize_ + sizeof(uint64_t);
    msgLen() = bufSize_;
  }

 private:
  uint32_t imgSize_;
  uint32_t bufSize_;

  bool parseData(char *pMsg, size_t len, mtParsed &parsed) override {
    if (len != bufSize_) return false;
    parsed.img = cv::Mat(cv::Size(width_, height_), CV_8UC1, pMsg).clone();
    parsed.t = (*(uint64_t *) (pMsg + imgSize_)) * 1e-9;
    return true;
  }
};

struct BottomImage {
  uint8_t img_data[320*240];
  double timestamp_sec;
};

class BottomServer : public nnstation::NanoServer<BottomImage> {
 public:
  bool send(const mtParsed &parsed) override {
    return sendMsg(&parsed, sizeof(mtParsed));
  }
};

}
