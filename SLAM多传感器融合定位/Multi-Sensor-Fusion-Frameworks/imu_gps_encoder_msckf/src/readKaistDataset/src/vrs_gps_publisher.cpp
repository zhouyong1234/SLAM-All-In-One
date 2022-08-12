#include <bits/stdc++.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <nav_msgs/Path.h>
#include <LocalCartesian.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

const int vrs_gps_rate=1;
const string vrs_gps_file_path="/home/touchair/kaist_urban_dataset/urban32/sensor_data/vrs_gps.csv";
const double kToSecond = 1e-9;
Eigen::Vector3d init_lat_lon_alt;
nav_msgs::Path vrs_gps_path;

Eigen::Vector3d LatLonAlt2ENU(Eigen::Vector3d lat_lon_alt){ 
	static GeographicLib::LocalCartesian local_cartesian;
	local_cartesian.Reset(init_lat_lon_alt.x(), init_lat_lon_alt.y(), init_lat_lon_alt.z());
	Eigen::Vector3d enu_pose;
    local_cartesian.Forward( lat_lon_alt.x(), lat_lon_alt.y(), lat_lon_alt.z(),
		enu_pose.data()[0], enu_pose.data()[1], enu_pose.data()[2]);
	return enu_pose;
}

void enu2nav_msgs(Eigen::Vector3d& enu_pose){
    vrs_gps_path.header.frame_id = "world";
    vrs_gps_path.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = vrs_gps_path.header;

    pose.pose.position.x = enu_pose.x();
    pose.pose.position.y = enu_pose.y();
    pose.pose.position.z = enu_pose.z();

    const Eigen::Quaterniond Q_w_i(Eigen::Matrix3d::Identity());
    pose.pose.orientation.x = Q_w_i.x();
    pose.pose.orientation.y = Q_w_i.y();
    pose.pose.orientation.z = Q_w_i.z();
    pose.pose.orientation.w = Q_w_i.w();

    vrs_gps_path.poses.push_back(pose);
	return ;
}

int main(int argc,char** argv){
	ros::init(argc, argv, "vrs_gps_publisher");

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::NodeHandle n;

    ros::Publisher vrs_gps_info_pub = n.advertise<nav_msgs::Path>("/vrs_gps_info", 10);

    ros::Rate loop_rate(vrs_gps_rate);

	std::queue<vector<double>> gps_queue;
	std::ifstream gps_file(vrs_gps_file_path);
	string line_str,value_str;
	std::vector<std::string> line_data_vec;
	while (std::getline(gps_file, line_str)){
		line_data_vec.clear();
		line_data_vec.reserve(18);
		std::stringstream ss(line_str);
		while (std::getline(ss, value_str, ','))
			line_data_vec.push_back(value_str);
		vector<double> lat_lon_alt;
		lat_lon_alt.push_back(std::stod(line_data_vec[1]));
		lat_lon_alt.push_back(std::stod(line_data_vec[2]));
		lat_lon_alt.push_back(std::stod(line_data_vec[5]));
		gps_queue.push(lat_lon_alt);
	}
	init_lat_lon_alt << gps_queue.front()[0], gps_queue.front()[1], gps_queue.front()[2];
	gps_queue.pop();

	while (ros::ok()){
		if(!gps_queue.empty()){
			vector<double> gps_data = gps_queue.front();
			Eigen::Vector3d lat_lon_alt_vec(gps_data[0],gps_data[1],gps_data[2]);
			Eigen::Vector3d enu_pose = LatLonAlt2ENU(lat_lon_alt_vec);
			enu2nav_msgs(enu_pose);
			vrs_gps_info_pub.publish(vrs_gps_path);
			gps_queue.pop();
		}
		loop_rate.sleep();
	}

	return 0;
}