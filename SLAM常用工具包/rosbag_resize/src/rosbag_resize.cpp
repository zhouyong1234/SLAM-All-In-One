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
 


 
int main(int argc, char** argv)
{
      // Initialize ROS
    ros::init (argc, argv, "rosbag_resize");
    ros::NodeHandle nh;
 
    rosbag::Bag bag;
    rosbag::Bag bag_write;
    bag.open("/home/touchair/bag/gnss_cam_imu.bag", rosbag::bagmode::Read); //打开一个bag文件
    bag_write.open("/home/touchair/bag/gnss_cam_imu_downsize.bag", rosbag::bagmode::Write); 

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/image", 1000);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);
    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps", 1000);
    
    std::vector<std::string> topics; //设置需要遍历的topic
    topics.push_back(std::string("/camera/image_raw"));         
    topics.push_back(std::string("/android/imu"));        
    topics.push_back(std::string("/android/fix"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));; // 读指定的topic，如果全读，第二个参数不写，如下
    //rosbag::View view_all(view); //读取全部topic
    
    
    foreach(rosbag::MessageInstance const m, view) //用foreach遍历所有帧数据，每个messageInstance就是一帧数据
    {

        std::string topic = m.getTopic();
        cv::Mat img_resize;



        if(topic == "/camera/image_raw")
        {
            sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
            if(image != NULL)
            {
                // std::cout << image->header.stamp << std::endl;
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image,  sensor_msgs::image_encodings::TYPE_8UC3);
                cv::Mat img_ = cv_ptr->image;
                cv::resize(img_, img_resize, cv::Size(640, 480));
                // std::cout << img_resize.size() << std::endl;
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_resize).toImageMsg();
                msg->header.stamp = image->header.stamp;
                msg->header.frame_id = image->header.frame_id;
                std::cout << "img stamp: " << msg->header.stamp << std::endl;
                // image_pub.publish(*msg);
                bag_write.write("/image", msg->header.stamp, msg);
            }
            

        }
        
        
        if(topic == "/android/imu")
        {
            sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
            if(imu != NULL)
            {
                std::cout << "imu stamp: " << imu->header.stamp << std::endl;
                // imu_pub.publish(*imu);
                bag_write.write("/imu", imu->header.stamp, imu);
            }
            
        }


        if(topic == "/android/fix")
        {
            sensor_msgs::NavSatFix::ConstPtr gps = m.instantiate<sensor_msgs::NavSatFix>();
            if(gps != NULL)
            {
                std::cout << "gps stamp: " << gps->header.stamp << std::endl;
                // gps_pub.publish(*gps);
                bag_write.write("/gps", gps->header.stamp, gps);
            }
        }



        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    }
    bag.close();

    return 0;
}