#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "Config.hpp"
#include "Camera.hpp"

vio::Camera* cam = nullptr;
cv::Mat map1,map2;
double scale = 2.;
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  cv::Mat image = cv_ptr->image.clone();
  cv::Mat undistImg = cam->undistImage(image,map1,map2);
  cv::resize(image,image,cv::Size(scale * image.cols, scale * image.rows));
  cv::hconcat(image,undistImg,image);
  cv::imshow("undist image",image);
  cv::waitKey(1);
  std::cout << "image size = " << undistImg.cols << "," << undistImg.rows << std::endl;  
}


int main(int argc,char** argv) {
  ros::init(argc,argv,"test_camera");
  ros::NodeHandle nh("~");
  std::string topic_image,cfgFile;
  nh.param<std::string>("topic_image", topic_image, "/image");
  nh.param<std::string>("config_file", cfgFile, " ");
  std::cout << "topic_image: " << topic_image << std::endl;
  vio::Config* cfg = new vio::Config(cfgFile);
  cam = new vio::Camera(cfg);
  cam->getRectifyMap(map1,map2,scale);
  ros::Subscriber imgSub = nh.subscribe<sensor_msgs::Image>(topic_image,10,imageCallback);
  ros::spin();
  return 0;
}
