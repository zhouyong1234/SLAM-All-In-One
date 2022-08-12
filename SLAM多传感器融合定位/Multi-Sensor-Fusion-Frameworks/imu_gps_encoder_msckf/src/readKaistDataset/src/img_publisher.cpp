#include <bits/stdc++.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;

const int img_rate=10;
const string img_file_="/home/touchair/kaist_urban_dataset/urban32/image/stereo_left/";
const string img_file_path="/home/touchair/kaist_urban_dataset/urban32/sensor_data/stereo_stamp.csv";
const string data_stamp_file_path="/home/touchair/kaist_urban_dataset/urban32/sensor_data/data_stamp.csv";
const double kToSecond = 1e-9;


ros::Time TimestampToRosTime(std::string timestamp)
{
    size_t len = timestamp.length();
    std::string sec_string = timestamp.substr(0, 10);
    std::string nsec_string = timestamp.substr(10,len);

    // std::cout << sec_string << ", " << nsec_string << std::endl;

    while(nsec_string.length() < 9){
        nsec_string += "0";
    }
    return ros::Time(std::stoi(sec_string),std::stoi(nsec_string));
}

int main(int argc,char** argv){
	ros::init(argc, argv, "img_publisher");

    ros::NodeHandle n;

    ros::Publisher img_info_pub = n.advertise<sensor_msgs::Image>("/img_info", 10);

	ros::Rate loop_rate(img_rate);

	std::queue<string> img_queue;
	std::ifstream img_file(img_file_path);
	string line_str;
	while (std::getline(img_file, line_str))
		img_queue.push(line_str);
	
	while (ros::ok()){
		if(!img_queue.empty()){
			string img_file = img_file_ + img_queue.front() + ".png";
			cv::Mat raw_image = cv::imread(img_file, CV_LOAD_IMAGE_ANYDEPTH);
            cv::Mat color_img;
            cv::cvtColor(raw_image, color_img, cv::COLOR_BayerRG2RGB);
            cv::Mat gray_img;
            cv::cvtColor(color_img, gray_img, cv::COLOR_RGB2GRAY);
			sensor_msgs::Image img_msg;
			cv_bridge::CvImage img_bridge;
			std_msgs::Header header;
			header.stamp = TimestampToRosTime(img_queue.front());
			header.frame_id = "/stereo_left";
			img_queue.pop();
			img_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::MONO8,gray_img);
			img_bridge.toImageMsg(img_msg);
			img_info_pub.publish(img_msg);
		}
		loop_rate.sleep();
	}
	return 0;
}