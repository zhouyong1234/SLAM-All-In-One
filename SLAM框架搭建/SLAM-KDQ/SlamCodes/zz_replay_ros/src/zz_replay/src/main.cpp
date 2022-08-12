#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include "BottomImage.hpp"
#include "IMU.hpp"
using namespace std;

string IMU_TOPIC = "/imu";
string IMAGE_TOPIC = "/image";
string REPLAY_FOLDER = "/";
double SKIP_TIME_S = 0;
int WRITE_BAG_FLG = false;
rosbag::Bag bagOut;
mutex lck;

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
    fs["skip_time"] >> SKIP_TIME_S;
    fs["write_bag"] >> WRITE_BAG_FLG;
    if(WRITE_BAG_FLG) {
        string rosBagName = REPLAY_FOLDER + "/dataset.bag";
        bagOut.open(rosBagName,rosbag::bagmode::Write);
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
    pub_img = n.advertise<sensor_msgs::Image>(IMAGE_TOPIC,1000);
    pub_imu = n.advertise<sensor_msgs::Imu>(IMU_TOPIC,2000);
    printf("Input replay folder:%s\n", REPLAY_FOLDER.c_str());
    nnstation::BottomClient bottomClient;
    nnstation::ImuClient imuClient;
    bottomClient.setImageSize(320, 240);
    bottomClient.subscribe([&](const nnstation::BottomClient::mtParsed &parsed) {
        auto current_t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        auto delay = static_cast<float>(current_t - parsed.t);
        printf("[Image] Get %12.6f at %12.6f, delay = %7.3fms\n", parsed.t, current_t, delay * 1e3f);
        publishImage(parsed.t,parsed.img,pub_img);
     
    });
    imuClient.subscribe([&](const nnstation::ImuClient::mtParsed &parsed) {
        for (const auto &info : parsed)
        {
            usleep(1);
            printf("[IMU] Get %12.6f imu data!\n", info.t);
            publishImu(info,pub_imu);
            //rovioNode.imuCallback(info);
        }
    });

    if (!REPLAY_FOLDER.empty())
    {
        auto t1 = DBL_MAX, t2 = DBL_MAX; //, t3 = DBL_MAX;
        bottomClient.getOldestReplayTime(REPLAY_FOLDER + "/img.rec", t1);
        imuClient.getOldestReplayTime(REPLAY_FOLDER + "/imu.rec", t2);
        double max_data_t = std::max(t1, t2);
        double start_data_t = std::max(max_data_t, SKIP_TIME_S);
        printf("Oldest bottom time: %20.9f\n", t1);
        printf("Oldest imu time: %20.9f\n", t2);
        std::cout << "start replay since: " << start_data_t << std::endl;
        auto start_real_t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        bottomClient.startReplay(REPLAY_FOLDER + "/img.rec", start_real_t, start_data_t);
        imuClient.startReplay(REPLAY_FOLDER + "/imu.rec", start_real_t, start_data_t);
    }
    sleep(UINT32_MAX);
    return 0;
}