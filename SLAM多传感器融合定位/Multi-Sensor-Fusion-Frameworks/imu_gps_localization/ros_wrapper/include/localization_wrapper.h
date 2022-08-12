#pragma once

#include <fstream>
#include <memory>
#include <deque>
#include <algorithm>
#include <numeric>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include "imu_gps_localizer/imu_gps_localizer.h"


class LocalizationWrapper {
public:
    LocalizationWrapper(ros::NodeHandle& nh);
    ~LocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);

private:
    void LogState(const ImuGpsLocalization::State& state);
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data, const ImuGpsLocalization::State& state);

    void SavePose(std::ofstream &ofs, const Eigen::Matrix4d &pose);

    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state);


    bool CheckGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data);
    
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_position_sub_;
    ros::Publisher state_pub_;
    ros::Publisher gps_path_pub_;
    ros::Publisher gps_noise_pub_;

    bool inited_ = false;

    bool first_ = false;


    double gps_timestamp_;

    double delta_x;
    double delta_y;

    double sum_state_delta_x;
    double sum_state_delta_y;

    std::deque<double> delta_x_;
    std::deque<double> delta_y_;

    std::deque<double> state_delta_x_;
    std::deque<double> state_delta_y_;

    std::ofstream file_state_;
    std::ofstream file_gps_;

    nav_msgs::Path ros_path_;
    nav_msgs::Path gps_path_;

    Eigen::Vector3d last_gps_posetion_;
    Eigen::Vector3d curr_gps_posetion_;

    Eigen::Vector3d last_state_posetion_;
    Eigen::Vector3d curr_state_posetion_;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
};