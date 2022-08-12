//
// Created by kdq on 2021/5/21.
//
#ifndef D435I_SDK_D435ICONFIG_HPP
#define D435I_SDK_D435ICONFIG_HPP

struct D435IStreamConfigParam {
  D435IStreamConfigParam() {
    printf("[D435I-DefaultParameters]:default values are set according to realsense-viwer!\n");
    streamType = infrared;
    width = 640;
    height = 480;
    framerate = 30;
    auto_exposure = 1;
    exposure_time = 5000.0;
    exposure_gain = 80.;
    ae_point = 2000;
  }
  void print() const{
    std::cout << "===============D435I-StreamConfig==============" << std::endl;
    std::cout << "streamType = " << streamType << std::endl;
    std::cout << "width = " << width << std::endl;
    std::cout << "height = " << height << std::endl;
    std::cout << "framerate = " << framerate << std::endl;
    std::cout << "auto_exposure = " << auto_exposure << std::endl;
    std::cout << "exposure_time = " << exposure_time << std::endl;
    std::cout << "exposure_gain = " << exposure_gain << std::endl;
    std::cout << "ae_point = " << ae_point << std::endl;
  }
  enum Stream {
    infrared,
//    rgb,
//    depth,
  }; //当前仅支持双目
  int streamType;
  int width;
  int height;
  int framerate;
  int auto_exposure;
  float exposure_time; //us
  float exposure_gain;
  float ae_point; //this value work when auto-exposure is enabled,present mean intensity of image
};

struct D435IIMUConfigParam {
  D435IIMUConfigParam() {
    enable = 1;
    sync_enable = 1;
  }
  void print() const{
    std::cout << "===============D435I-IMUConfig==============" << std::endl;
    std::cout << "enable: " << enable << std::endl;
    std::cout << "sync_enable: " << sync_enable << std::endl;
  }
  int enable;
  int sync_enable;
};

struct StereoStream{
  StereoStream():timestamp(0.){};
  double timestamp;
  cv::Mat leftImg;
  cv::Mat rightImg;
};
struct ImuStream {
  ImuStream():timestamp(0.),gyr{0.},acc{0.} {};
  double timestamp;
  double gyr[3];
  double acc[3];
};

#endif //D435I_SDK_D435ICONFIG_HPP
