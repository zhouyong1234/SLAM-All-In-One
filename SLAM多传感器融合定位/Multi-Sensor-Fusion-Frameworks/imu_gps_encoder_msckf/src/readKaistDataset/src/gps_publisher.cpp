#include <bits/stdc++.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <read_kaist_dataset/gps.h>

using namespace std;

const int gps_rate=10;
const string gps_file_path="/home/touchair/kaist_urban_dataset/urban32/sensor_data/gps.csv";
const string data_stamp_file_path="/home/touchair/kaist_urban_dataset/urban32/sensor_data/data_stamp.csv";
const double kToSecond = 1e-9;

int main(int argc,char** argv){
	ros::init(argc, argv, "gps_publisher");

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::NodeHandle n;

    ros::Publisher gps_info_pub = n.advertise<read_kaist_dataset::gps>("/gps_info", 10);

    ros::Rate loop_rate(gps_rate);

	std::queue<read_kaist_dataset::gps> gps_queue;
	std::ifstream gps_file(gps_file_path);
	string line_str,value_str;
	std::vector<std::string> line_data_vec;
	while (std::getline(gps_file, line_str)){
		line_data_vec.clear();
		line_data_vec.reserve(13);
		std::stringstream ss(line_str);
		while (std::getline(ss, value_str, ','))
			line_data_vec.push_back(value_str);
		read_kaist_dataset::gps gps_msg;
		gps_msg.timeStamp = std::stod(line_data_vec[0]) * kToSecond;
		gps_msg.latitude = std::stod(line_data_vec[1]);
        gps_msg.longitude = std::stod(line_data_vec[2]);
		gps_msg.altitude = std::stod(line_data_vec[3]);
        gps_msg.cov00 = std::stod(line_data_vec[4]);
		gps_msg.cov01 = std::stod(line_data_vec[5]);
		gps_msg.cov02 = std::stod(line_data_vec[6]);
		gps_msg.cov10 = std::stod(line_data_vec[7]);
		gps_msg.cov11 = std::stod(line_data_vec[8]);
		gps_msg.cov12 = std::stod(line_data_vec[9]);
		gps_msg.cov20 = std::stod(line_data_vec[10]);
		gps_msg.cov21 = std::stod(line_data_vec[11]);
		gps_msg.cov22 = std::stod(line_data_vec[12]);
		gps_queue.push(gps_msg);
	}

	while (ros::ok()){
		if(!gps_queue.empty()){
			gps_info_pub.publish(gps_queue.front());
			gps_queue.pop();
		}
		loop_rate.sleep();
	}

	return 0;
}