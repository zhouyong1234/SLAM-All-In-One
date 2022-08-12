#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include "rovio.pb.h"
#include "image.pb.h"
#include "replaykit.h"

using namespace std;

string IMU_TOPIC = "/imu";
string IMAGE_TOPIC = "/image";
string REPLAY_FOLDER = "./";
string OUTPUT_PATH = "./";
double SKIP_TIME_S = 0;
double SPEED_K = 1.0;
int WRITE_BAG_FLG = false;
rosbag::Bag bagOut;
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
    if (WRITE_BAG_FLG)
    {
      string rosBagName = OUTPUT_PATH + "/dataset.bag";
      bagOut.open(rosBagName, rosbag::bagmode::Write);
    }
}

void publishImu(const TimedImuData &imuData,ros::Publisher& publisher)
{
  std::lock_guard<std::mutex> lock(lck);
  sensor_msgs::Imu imuMsg;
  imuMsg.angular_velocity.x = imuData.gyr.x();
  imuMsg.angular_velocity.y = imuData.gyr.y();
  imuMsg.angular_velocity.z = imuData.gyr.z();
  imuMsg.linear_acceleration.x = imuData.acc.x();
  imuMsg.linear_acceleration.y = imuData.acc.y();
  imuMsg.linear_acceleration.z = imuData.acc.z();
  imuMsg.header.stamp.fromSec(imuData.t);
  if(WRITE_BAG_FLG){
    bagOut.write(IMU_TOPIC,ros::Time::now(),imuMsg);
  } else {
    publisher.publish(imuMsg);
  }

}
void publishImage(const double t,const cv::Mat& img,ros::Publisher& publisher)
{
  //KDQ:如果不上锁很可能被imu的callback打断,导致内存泄露
  std::lock_guard<std::mutex> lock(lck);
  std_msgs::Header timestamp;
  timestamp.stamp.fromSec(t);
  cv_bridge::CvImage imgMsg(timestamp,"mono8",img);
  if(WRITE_BAG_FLG){
    bagOut.write(IMAGE_TOPIC,ros::Time::now(),imgMsg.toImageMsg());
  }else{
    sensor_msgs::ImagePtr msgPtr = imgMsg.toImageMsg();
    publisher.publish(msgPtr);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zz_replay");
  ros::NodeHandle n("~");
  cout << "para:" << argc << endl;
  if (argc < 2)
  {
      ROS_ERROR("Please input config file path!");

      ros::shutdown();
      return -1;
  }
  ros::Publisher pub_img,pub_imu;
  string configFile = argv[1];
  //n.getParam("config_file",configFile);
  readParameters(configFile);
  ReplayKitType replaykit;
  pub_img = n.advertise<sensor_msgs::Image>(IMAGE_TOPIC,1000);
  pub_imu = n.advertise<sensor_msgs::Imu>(IMU_TOPIC,2000);
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
    publishImage(bottomImage.timestamp() + exp_time * 0.5,im,pub_img);
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
      publishImu(element,pub_imu);
    }
  });
  zz::replaykit::FileReplayReader<ReplayKitType> reader(REPLAY_FOLDER, replaykit, SPEED_K);
  reader.SetStartTime(SKIP_TIME_S);
  std::thread replay_thread([&]() {
    replaykit.Start();
  });
  reader.Start();
  printf("Rovio replay finished!\n");
  while(ros::ok()) {
    usleep(1000);
  }
  ros::spin();
  return 0;
}