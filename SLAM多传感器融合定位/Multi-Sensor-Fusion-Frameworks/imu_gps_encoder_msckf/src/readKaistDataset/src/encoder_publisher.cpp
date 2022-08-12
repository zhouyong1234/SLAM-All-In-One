#include <bits/stdc++.h>
#include <read_kaist_dataset/encoder.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
using namespace std;

const int wheel_encoder_rate=100;
const string encoder_file_path="/home/touchair/kaist_urban_dataset/urban32/sensor_data/encoder.csv";
const string data_stamp_file_path="/home/touchair/kaist_urban_dataset/urban32/sensor_data/data_stamp.csv";
const double kToSecond = 1e-9;

int main(int argc,char** argv){
	ros::init(argc, argv, "encoder_publisher");

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::NodeHandle n;

    ros::Publisher encoder_info_pub = n.advertise<read_kaist_dataset::encoder>("/encoder_info", 10);

    ros::Rate loop_rate(wheel_encoder_rate);

	std::queue<read_kaist_dataset::encoder> encoder_queue;
	std::ifstream encoder_file(encoder_file_path);
	string line_str,value_str;
	std::vector<std::string> line_data_vec;
	while (std::getline(encoder_file, line_str)){
		line_data_vec.clear();
		line_data_vec.reserve(3);
		std::stringstream ss(line_str);
		while (std::getline(ss, value_str, ','))
			line_data_vec.push_back(value_str);
		read_kaist_dataset::encoder encoder_msg;
		encoder_msg.timeStamp = std::stod(line_data_vec[0]) * kToSecond;
		encoder_msg.leftEncoder = std::stod(line_data_vec[1]);
		encoder_msg.rightEncoder = std::stod(line_data_vec[2]);
		encoder_queue.push(encoder_msg);
	}

	while (ros::ok()){
		if(!encoder_queue.empty()){
			encoder_info_pub.publish(encoder_queue.front());
			ROS_DEBUG("Publish encoder Info: time:%f  left:%f  right:%f", 
				encoder_queue.front().timeStamp, encoder_queue.front().leftEncoder, encoder_queue.front().rightEncoder);
			encoder_queue.pop();
		}
		loop_rate.sleep();
	}

	return 0;
}