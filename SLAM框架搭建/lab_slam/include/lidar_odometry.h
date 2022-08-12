//
// Created by jin on 2021/5/25.
//

#ifndef LABLOAM_LIDAR_ODOMETRY_H
#define LABLOAM_LIDAR_ODOMETRY_H

#include <ros/ros.h>
#include "features_register.h"
#include "data_defination.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
//#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <pcl/common/transforms.h>
#include "key_frame.hpp"

class LidarOdometry{
public:
    LidarOdometry();

    bool work(const DataGroupPtr& data_group, const std::vector<KeyFramePtr>& key_frames, KeyFramePtr& new_frame);

    void publish(std_msgs::Header h);

    void publishOpt(std_msgs::Header h);

    void setCurrentPoseOpt(const Eigen::Affine3d& pose) {
        current_pose_opt_ = pose;
    }

    void setOdoDrift(const Eigen::Affine3d& drift){
        odo_drift_ = drift;
    }
private:
    bool deskew_ = false;// aloam开启这个效果也不好
    bool use_const_velo_ = true;
    Scan2ScanMatcher scan2scan_matcher_;
    Eigen::Affine3d last_scan2scan_pose_ = Eigen::Affine3d::Identity();
    Eigen::Affine3d current_pose_ = Eigen::Affine3d::Identity();
    PointCloudXYZIPtr last_corners_ = nullptr;
    PointCloudXYZIPtr last_planes_ = nullptr;
//    pcl::KdTreeFLANN<PointXYZI>::Ptr last_corner_tree_;
//    pcl::KdTreeFLANN<PointXYZI>::Ptr last_plane_tree_;
    bool is_initialized = false;
    ros::Publisher corners_pub_;
    ros::Publisher planes_pub_;
    ros::Publisher odo_pub_;
    ros::Publisher odo_path_pub_;

    Scan2MapMatcher scan2map_mather_;
    Eigen::Affine3d last_history_pose_ = Eigen::Affine3d::Identity();
    double accumulate_dis_ = 0.0;
//    std::vector<std::shared_ptr<KeyFrame>> key_frames_;
    PointCloudXYZIPtr surrounding_corner_map_  = nullptr;
    PointCloudXYZIPtr surrounding_plane_map_ = nullptr;
    PointCloudXYZIPtr surrounding_map_ = nullptr;
    Eigen::Affine3d odo_drift_ = Eigen::Affine3d::Identity();
    Eigen::Affine3d current_pose_opt_ = Eigen::Affine3d::Identity();
    ros::Publisher surrounding_map_pub_;
    ros::Publisher origin_cloud_repub_;
    ros::Publisher opt_odo_pub_;
    ros::Publisher opt_odo_path_pub_;
};

#endif //LABLOAM_LIDAR_ODOMETRY_H
