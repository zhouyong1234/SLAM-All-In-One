#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include "d435iConfig.hpp"

class D435I {
 public:  
  D435I(){};

  D435I(const std::string &cfgFile);

  void start();


  bool getInfraredImages(double& timestamp,cv::Mat& leftImg,cv::Mat& rightImg) const;

  bool getD435IStreamDatas(StereoStream& stereoInfos,ImuStream& imuInfos) const;

 private:
  rs2::pipeline pipe_;
  D435IStreamConfigParam stereoConfigParam_;
  D435IIMUConfigParam imuConfigParam_;
};
