#include <bits/stdc++.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <read_kaist_dataset/imu.h>
using namespace std;

const int imu_rate=100;
const string imu_file_path="/home/touchair/kaist/sensor_data/xsens_imu.csv";
const string data_stamp_file_path="/home/touchair/kaist/sensor_data/data_stamp.csv";
const double kToSecond = 1e-9;

int main(int argc,char** argv){
	ros::init(argc, argv, "imu_publisher");

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::NodeHandle n;

    ros::Publisher imu_info_pub = n.advertise<read_kaist_dataset::imu>("/imu_info", 10);

    ros::Rate loop_rate(imu_rate);

	std::queue<read_kaist_dataset::imu> imu_queue;
	std::ifstream imu_file(imu_file_path);
	string line_str,value_str;
	std::vector<std::string> line_data_vec;
	while (std::getline(imu_file, line_str)){
		line_data_vec.clear();
		line_data_vec.reserve(17);
		std::stringstream ss(line_str);
		while (std::getline(ss, value_str, ','))
			line_data_vec.push_back(value_str);
		read_kaist_dataset::imu imu_msg;
		imu_msg.timeStamp = std::stod(line_data_vec[0]) * kToSecond;
		imu_msg.gx = std::stod(line_data_vec[8]);
		imu_msg.gy = std::stod(line_data_vec[9]);
		imu_msg.gz = std::stod(line_data_vec[10]);
		imu_msg.ax = std::stod(line_data_vec[11]);
		imu_msg.ay = std::stod(line_data_vec[12]);
		imu_msg.az = std::stod(line_data_vec[13]);
		imu_queue.push(imu_msg);
	}

	while (ros::ok()){
		if(!imu_queue.empty()){
			imu_info_pub.publish(imu_queue.front());
			// ROS_INFO("time: %f",  imu_queue.front().timeStamp);
			imu_queue.pop();
		}
		loop_rate.sleep();
	}

	return 0;
}