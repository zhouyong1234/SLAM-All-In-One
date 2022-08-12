#include <list>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/bind.hpp>
#include "Camera.hpp"
#include "FeatureTracker.hpp"

using namespace vio;


void showAllFeature(const std::list<std::map<uint64_t,cv::Point2f>>& corners,cv::Mat &img,uint scale);
void monoCallback(sensor_msgs::ImageConstPtr msg,FeatureTracker* featTracker,FramePtr lastFrame);
int main(int argc,char **argv) {
  ros::init(argc,argv,"test_featureTracker");
  ros::NodeHandle nh("~");
  std::string cfgFile,imageTopic;
  nh.param<std::string>("config_file",cfgFile," ");
  nh.param<std::string>("topic_image",imageTopic,"cam0/image_raw");
  std::cout << "config :" << cfgFile << std::endl;
  Config* cfg = new Config(cfgFile);
  Camera* cam = new Camera(cfg);
  FeatureTracker *featTracker = new FeatureTracker(cfg);
  FramePtr lastF = nullptr;
  ros::Subscriber imgSub = nh.subscribe<sensor_msgs::Image>(imageTopic,10,boost::bind(&monoCallback,_1,featTracker,lastF));
  ros::spin();
  return 0;
}


void monoCallback(sensor_msgs::ImageConstPtr msg,FeatureTracker* featTracker,FramePtr lastFrame) {
  static std::list<std::map<uint64_t,cv::Point2f> > windowsFeatures;
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat image = cv_ptr->image.clone();
  double timestamp = cv_ptr->header.stamp.now().toSec();
  FramePtr curFrame(new Frame(timestamp,image));
  featTracker->detectAndTrackFeature(lastFrame,curFrame);
  lastFrame = curFrame;
  curFrame->imshowFeatures(2);
  std::map<uint64_t,cv::Point2f> curCorners = curFrame->getCornersCopy();
  windowsFeatures.push_back(curCorners);
  //showAllFeature(windowsFeatures,curFrame->image_,2);
  if (windowsFeatures.size() > 5) {
    windowsFeatures.pop_front();
  }
}

void showAllFeature(const std::list<std::map<uint64_t,cv::Point2f>> &corners,cv::Mat &img,uint scale) {
  cv::Mat colorImg;
  cv::cvtColor(img,colorImg,cv::COLOR_GRAY2BGR);
   
  std::map<uint64_t,cv::Point2f> cornerFront = corners.front();
  std::map<uint64_t,cv::Point2f> cornerBack = corners.back();
  std::cout << "corner front size = " << cornerFront.size() << " back size = " << cornerBack.size() << std::endl;
  for(auto it = cornerBack.begin(); it != cornerBack.end(); it++) {
    uint64_t id = it->first;
    std::cout << "id = " << id << std::endl;
    if (cornerFront.count(id)) {
      cv::Point2f pointBegin = cornerFront[id];
      cv::Point2f pointEnd = cornerBack[id];
      cv::line(colorImg,pointBegin,pointEnd,cv::Scalar(0,255,0));
    }
  }
  cv::resize(colorImg,colorImg,cv::Size(colorImg.cols * scale, colorImg.rows * scale));
  cv::imshow("track test",colorImg);
  cv::waitKey(1);
}