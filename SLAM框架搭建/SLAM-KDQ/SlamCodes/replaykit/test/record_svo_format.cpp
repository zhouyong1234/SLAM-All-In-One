#include <fstream>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include "rovio.pb.h"
#include "image.pb.h"
#include "replaykit/replaykit.h"
#include <opencv/cv.hpp>

using namespace std;


string IMU_TOPIC = "/imu";
string IMAGE_TOPIC = "/image";
string REPLAY_FOLDER = "./";
string OUTPUT_PATH = "./";
double SKIP_TIME_S = 0;
double SPEED_K = 1.0;
int WRITE_BAG_FLG = false;
mutex lck;

struct TimedImuData {
  double t;
  Eigen::Vector3d gyr;
  Eigen::Vector3d acc;
  Eigen::Vector3d init_vel;
  Eigen::Quaterniond init_quat;
  float proxi;
  bool steady_on_ground;
  bool servo_checking;
};

typedef ::zz::replaykit::ReplayKit<
    ::zz::replaykit::Topics<vision::BottomImage, rovio::InputInfoPack>,
    ::zz::replaykit::Commands<>>
    ReplayKitType;
void readParameters(const string configFile)
{
    cv::FileStorage fs(configFile,cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    fs["imu_topic"] >> IMU_TOPIC;
    fs["image_topic"] >> IMAGE_TOPIC;
    fs["replay_folder"] >> REPLAY_FOLDER;
    fs["output_path"] >> OUTPUT_PATH;
    fs["skip_time"] >> SKIP_TIME_S;
    fs["write_bag"] >> WRITE_BAG_FLG;
    fs["speed_k"] >> SPEED_K;
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    return -1;
  }
  string configFile = argv[1];
  readParameters(configFile);
  string picNameFileName = OUTPUT_PATH + "/data.txt";
  ofstream picNameFile(picNameFileName);
  ReplayKitType replaykit;
  printf("Input replay folder:%s\n", REPLAY_FOLDER.c_str());
    
  replaykit.Subscribe<0>([&](double now_time, const vision::BottomImage &bottomImage) {
    const cv::Mat im = cv::Mat(cv::Size(bottomImage.width(), bottomImage.height()), CV_8UC1,
                               (char *) bottomImage.image_buffer().c_str()).clone();
    auto delay = static_cast<float>(now_time - bottomImage.timestamp());
    auto exp_time = bottomImage.exposure_time();
    if (1) {
      printf("[Image] Get %12.6f at %12.6f, exp = %7.3fms, delay = %7.3fms\n",
             bottomImage.timestamp(), now_time, exp_time * 1e3f, delay * 1e3f);
    }
    //publishImage(bottomImage.timestamp() + exp_time * 0.5,im,pub_img);
    cv::imshow("img",im);
    cv::waitKey(1);
    string picName = to_string(bottomImage.timestamp() * 1e9);
    string picFinalName = OUTPUT_PATH + "/image/" + picName + ".png";
    cv::imwrite(picFinalName,im);
    picNameFile <<  picName  << endl;
  });

  replaykit.Subscribe<1>([&](double now_time, const rovio::InputInfoPack &info_pack) {
    TimedImuData element{};
    for (size_t i = 0; i < info_pack.info_size(); i++) {
      element.t = info_pack.info(i).t();
      element.gyr.x() = info_pack.info(i).gyr().x();
      element.gyr.y() = info_pack.info(i).gyr().y();
      element.gyr.z() = info_pack.info(i).gyr().z();
      element.acc.x() = info_pack.info(i).acc().x();
      element.acc.y() = info_pack.info(i).acc().y();
      element.acc.z() = info_pack.info(i).acc().z();
      element.init_quat.w() = info_pack.info(i).quat().w();
      element.init_quat.x() = info_pack.info(i).quat().x();
      element.init_quat.y() = info_pack.info(i).quat().y();
      element.init_quat.z() = info_pack.info(i).quat().z();
      element.init_vel.x() = 0.;
      element.init_vel.y() = 0.;
      element.init_vel.z() = 0.;
      element.proxi = info_pack.info(i).proxi();
//      printf("InputInfo: %11.6f, %7.3f,%7.3f,%7.3f, %7.3f,%7.3f,%7.3f, %7.4f,%7.4f,%7.4f,%7.4f\n",
//             parsed.t,
//             parsed.gyr[0], parsed.gyr[1], parsed.gyr[2],
//             parsed.acc[0], parsed.acc[1], parsed.acc[2],
//             parsed.init_quat.w(), parsed.init_quat.x(), parsed.init_quat.y(), parsed.init_quat.z());
     // publishImu(element,pub_imu);
    }
  });
  zz::replaykit::FileReplayReader<ReplayKitType> reader(REPLAY_FOLDER, replaykit, SPEED_K);
  reader.SetStartTime(SKIP_TIME_S);
  std::thread replay_thread([&]() {
    replaykit.Start();
  });
  reader.Start();
  printf("Rovio replay finished!\n");
  usleep(1000);
  return 0;
}