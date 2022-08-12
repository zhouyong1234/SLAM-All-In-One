#include "iostream"
#include "nnstation/BottomImage.hpp"
#include "cmdline.h"
#include "ImageAlignment.hpp"
#include "ConfigLoad.hpp"

void checkWarpMatrix(cv::Mat& warp12,cv::Mat& warp21,int alignModel) {
  cv::Mat warpCheck;
  if (alignModel == cv::MOTION_HOMOGRAPHY) {
    warpCheck = warp12 * warp21;
  } else {
    cv::Mat tm1 = cv::Mat::eye(3,3,CV_32F);
    cv::Mat tm2 = cv::Mat::eye(3,3,CV_32F);
    for (size_t i = 0; i < 2; i++) {
      for (size_t j = 0; j < 3; j++) {
        tm1.at<float>(i,j) = warp12.at<float>(i,j);
        tm2.at<float>(i,j) = warp21.at<float>(i,j);
      }
    }
    warpCheck = tm1 * tm2;
  }
  std::cout << "Warp12 = \n" << warp12 << std::endl;
  std::cout << "Warp21 = \n" << warp21 << std::endl;
  std::cout << "CheckWarp:\n" << warpCheck << std::endl;
}

int main(int argc,char** argv) {
  cmdline::parser parser;
  parser.add<std::string>("image_server_url", 'u', "image server url.", false, "tcp://127.0.0.1:11900");
  parser.add<std::string>("camera_config", 'f', "camera config file.", false, "rovio_cheerios.yaml");
  parser.add<int>("motion_model",'m',"image alignment model",false,1);
  parser.add<int>("iteration_number",'i',"iteration number of image alignment",false,50);
  parser.add<int>("pyramid_level",'l',"level of pyramid for image alignment",false,1);
  parser.add<bool>("histogram_enable",'h',"enable equalize histogram flg",false ,false);
  parser.parse_check(argc,argv);
  std::string imageUrl;
  std::string cameraConfigFile;
  int alignModel = cv::MOTION_EUCLIDEAN;
  int iterationNum = 40;
  int pyramidLevel = 1;
  bool histogramEnable = false;
  imageUrl = parser.get<std::string>("image_server_url");
  cameraConfigFile = parser.get<std::string>("camera_config");
  alignModel = parser.get<int>("motion_model");
  iterationNum = parser.get<int>("iteration_number");
  pyramidLevel = parser.get<int>("pyramid_level");
  histogramEnable = parser.get<bool>("histogram_enable");
  if (iterationNum <= 0) {
    iterationNum = 10;
  }
  ConfigLoad cfg(cameraConfigFile);
  Camera* cam = nullptr;
  if (cfg.readOK_) {
    cam = new Camera(cfg.K_,cfg.D_,cfg.fisheye_,cfg.width_,cfg.height_);
  }
  cv::Mat lastImg;
  ImageAlignment imgAlign(cam,alignModel,iterationNum,1e-10,pyramidLevel,histogramEnable);
  nnstation::BottomClient bottomClient;
  bottomClient.connect(imageUrl);
  bottomClient.subscribe([&](const vision::BottomImage &bottomImage) {
    cv::Mat im = cv::Mat(cv::Size(bottomImage.width(), bottomImage.height()), CV_8UC1,
                               (char *) bottomImage.image_buffer().c_str()).clone();
    if (!lastImg.empty()) {
      cv::Mat warp1 = imgAlign.Align(bottomImage.timestamp(),lastImg,im,true);
      cv::Mat warp2 = imgAlign.Align(bottomImage.timestamp(),im,lastImg);
      checkWarpMatrix(warp1,warp2,alignModel);
      std::cout << "WarpMatrix:" << bottomImage.timestamp() << ",";
      for (size_t i = 0; i < warp1.rows; i++) {
        for (size_t j = 0; j < warp1.cols; j++) {
          std::cout << warp1.at<float>(i,j) << ",";
        }
      }
      std::cout << std::endl;
    }
    lastImg = im.clone();
  });
  bottomClient.startRecv();
  sleep(UINT32_MAX);
  return 0;
}
