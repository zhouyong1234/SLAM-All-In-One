#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
 
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
 
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/CompressedImage.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <thread>
#include <iostream>


ros::Publisher image_pub;


void image_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    
    cv::Mat img_resize;

    // std::cout << image->header.stamp << std::endl;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg,  sensor_msgs::image_encodings::TYPE_8UC3);
    cv::Mat img_ = cv_ptr->image;
    cv::resize(img_, img_resize, cv::Size(640, 480));
    // std::cout << img_resize.size() << std::endl;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_resize).toImageMsg();
    msg->header.stamp = img_msg->header.stamp;
    msg->header.frame_id = img_msg->header.frame_id;
    std::cout << "img stamp: " << msg->header.stamp << std::endl;
    image_pub.publish(*msg);
    
} 

 
int main(int argc, char** argv)
{
      // Initialize ROS
    ros::init (argc, argv, "rostopic_resize");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 100, image_callback);
    image_pub = nh.advertise<sensor_msgs::Image>("/image", 1000);
    
    ros::spin();

    return 0;
}



