#pragma once

#include <opencv2/opencv.hpp>
#include "NanoStation.hpp"
#include <image.pb.h>
namespace nnstation {

class BottomClient : public NanoClient<vision::BottomImage> {
 public:
  typedef NanoClient<vision::BottomImage> Base;
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
    //msgLen() = bufSize_;
  }

 private:
  uint32_t imgSize_;
  uint32_t bufSize_;

  bool parseData(char *pMsg, size_t len, mtParsed &parsed) override {
    //std::cout << "WARNING! BottomClient::parseData() is not implemented!" << std::endl;
    parsed.ParseFromArray(pMsg, len);
    //if (len != bufSize_) return false;
    //parsed.img = cv::Mat(cv::Size(width_, height_), CV_8UC1, pMsg).clone();
    //parsed.t = (*(uint64_t *) (pMsg + imgSize_)) * 1e-9;
    return true;
  }
};

class BottomServer : public nnstation::NanoServer<vision::BottomImage> {
 public:
  bool send(const mtParsed &parsed) override {
    auto serialized = parsed.SerializeAsString();
    return sendMsg(serialized.c_str(), serialized.size());
  }
};

}
