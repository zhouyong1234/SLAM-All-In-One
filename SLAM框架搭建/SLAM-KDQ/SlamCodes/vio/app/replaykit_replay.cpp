//
// Created by kdq on 2021/6/14.
//
#include "iostream"
#include "rovio.pb.h"
#include "image.pb.h"
#include "replaykit.h"
#include "BottomImage.hpp"
#include "IMU.hpp"
#include "cmdline.h"
#include "Estimator.hpp"
#include "VizScene.hpp"

using namespace vio;
typedef ::zz::replaykit::ReplayKit<::zz::replaykit::Topics<vision::BottomImage, rovio::InputInfoPack>,::zz::replaykit::Commands<>> ReplayKitType;


int main(int argc,char **argv) {
  cmdline::parser parser;
  parser.add<double>("offset_time",'o',"image and imu time offset",false,0.);
  parser.add<std::string>("config_file",'c',"estimator config file",false,"../config/default.yaml");
  parser.add<int>("div_freq",'f',"divide freqency(hz)", false,1);
  parser.add<std::string>("replay_folder",'r',"replay folder",false,"");
  parser.add<double>("skip_time",'t',"replay skip time",false,0);
  parser.add<double>("replay_speed",'v',"replay speed",false,1.0);
  parser.parse_check(argc, argv);
  double timeOffset = 0.;
  std::string configFile;
  int divideFreq = 1;
  std::string replayFolder;
  double replaySkipTime = 0.;
  double replaySpeed = 1.0;
  timeOffset = parser.get<double>("offset_time");
  configFile = parser.get<std::string>("config_file");
  divideFreq = parser.get<int>("div_freq");
  replayFolder = parser.get<std::string>("replay_folder");
  replaySkipTime = parser.get<double>("skip_time");
  replaySpeed = parser.get<double>("replay_speed");

  Estimator estimator(configFile);
  const CameraPtr cam = estimator.getCameraPtr();
  VizScene vizWindow(&estimator,0.05);

  ReplayKitType replaykit;
  int imageCount = 0;
  replaykit.Subscribe<0>([&](double now_time, const vision::BottomImage &bottomImage) {
    cv::Mat im = cv::Mat(cv::Size(bottomImage.width(), bottomImage.height()), CV_8UC1,
                         (char *) bottomImage.image_buffer().c_str()).clone();
    auto exp_time = bottomImage.exposure_time();
    imageCount++;
    if (imageCount == divideFreq) {
      imageCount = 0;
//      printf("[Image] Get %12.6f at %12.6f, exp = %7.3fms\n",
//             bottomImage.timestamp(), now_time, exp_time * 1e3f);
      vio::FramePtr frame(new Frame(bottomImage.timestamp() + exp_time * 0.5 + timeOffset, im,cam));
      estimator.update(frame, true);
    }
  });
  std::ofstream imuRecord("imu.csv",std::ios::out);
  imuRecord << "t,ax,ay,az,gx,gy,gz" << std::endl;
  Eigen::Matrix3d RNU;
  RNU << 0., 1.0, 0.,
         -1.0, 0.0, 0.,
         0.,  0.0, 1.0;
  replaykit.Subscribe<1>([&](double now_time, const rovio::InputInfoPack &info_pack) {
    static double last_T = 0;
    for (size_t i = 0; i < info_pack.info_size(); i++) {
      const rovio::InputInfo info = info_pack.info(i);
      double timestamp = info.t();
      Eigen::Vector3d acc(info.acc().x(),info.acc().y(),info.acc().z());
      Eigen::Vector3d gyr(info.gyr().x(),info.gyr().y(),info.gyr().z());
      Eigen::Quaterniond qwc(info.quat().w(),info.quat().x(),info.quat().y(),info.quat().z());
      //debug
      //acc = RNU * qwc.toRotationMatrix() * acc;
      imuRecord << timestamp << "," << acc.x() << "," << acc.y() << "," << acc.z() << "," << gyr.x() << "," << gyr.y() << "," << gyr.z() << std::endl;
      estimator.updateImuMeas(timestamp,IMU(timestamp,acc,gyr));
    }
  });
  zz::replaykit::FileReplayReader<ReplayKitType> reader(replayFolder, replaykit, replaySpeed);
  reader.SetStartTime(replaySkipTime);
  std::thread replay_thread([&]() {
    replaykit.Start();
  });
  reader.Start();
  printf("Rovio replay finished!\n");
  return 0;
};