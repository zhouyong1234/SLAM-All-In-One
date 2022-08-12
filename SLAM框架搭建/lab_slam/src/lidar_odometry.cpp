//
// Created by jin on 2021/5/25.
//

#include "lidar_odometry.h"
#include <iostream>
#include "data_defination.hpp"

LidarOdometry::LidarOdometry(): scan2scan_matcher_(8, 2, deskew_,4), scan2map_mather_(8, 1){
//    current_pose_.setIdentity();
    last_corners_.reset(new PointCloudXYZI);
    last_planes_.reset(new PointCloudXYZI),
    surrounding_corner_map_.reset(new PointCloudXYZI);
    surrounding_plane_map_.reset(new PointCloudXYZI);
    surrounding_map_.reset(new PointCloudXYZI);

    ros::NodeHandle nh("~");
    corners_pub_ = nh.advertise<sensor_msgs::PointCloud2>("odo/less_corner_points", 1);
    planes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("odo/less_plane_points", 1);
    odo_pub_ = nh.advertise<nav_msgs::Odometry>("odo/lidar_odo", 1);
    odo_path_pub_ = nh.advertise<nav_msgs::Path>("odo/path", 1);
    opt_odo_pub_ = nh.advertise<nav_msgs::Odometry>("odo/opt_odo", 1);
    opt_odo_path_pub_ = nh.advertise<nav_msgs::Path>("odo/opt_path", 1);
    surrounding_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("odo/surrounding_map_points", 1);
    origin_cloud_repub_ = nh.advertise<sensor_msgs::PointCloud2>("odo/origin_cloud", 1);
    ROS_INFO("Lidar odo has been created...");
}

//
void LidarOdometry::publish(std_msgs::Header h) {
    // 发布tf
    static tf::TransformBroadcaster tb;
    tf::Transform trans;
    Eigen::Vector3d t(current_pose_.translation().matrix());
    trans.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    Eigen::Quaterniond q(current_pose_.rotation());
    tf::Quaternion tf_q;
    tf_q.setW(q.w());
    tf_q.setX(q.x());
    tf_q.setY(q.y());
    tf_q.setZ(q.z());
    trans.setRotation(tf_q);
    tb.sendTransform(tf::StampedTransform(trans, h.stamp, "map", "odo"));
    // 发布odo
    nav_msgs::Odometry odo_msg;
    odo_msg.header.stamp = h.stamp;
    odo_msg.header.frame_id = "map";
    odo_msg.child_frame_id = "odo";
    odo_msg.pose.pose.position.x = t.x();
    odo_msg.pose.pose.position.y = t.y();
    odo_msg.pose.pose.position.z = t.z();
    odo_msg.pose.pose.orientation.w = q.w();
    odo_msg.pose.pose.orientation.x = q.x();
    odo_msg.pose.pose.orientation.y = q.y();
    odo_msg.pose.pose.orientation.z = q.z();
    odo_pub_.publish(odo_msg);
    // 发布路径
    static nav_msgs::Path path;
    path.header.frame_id = "map";
    geometry_msgs::PoseStamped p;
    p.header.stamp = h.stamp;
    p.header.frame_id = "map";
    p.pose.position.x = t.x();
    p.pose.position.y = t.y();
    p.pose.position.z = t.z();
    p.pose.orientation.w = q.w();
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    path.poses.emplace_back(p);
    odo_path_pub_.publish(path);
//    // 发布点云
//    sensor_msgs::PointCloud2 tmp_msg;
//    pcl::toROSMsg(*last_corners_, tmp_msg);
//    tmp_msg.header = h;
//    corners_pub_.publish(tmp_msg);
//    pcl::toROSMsg(*last_planes_, tmp_msg);
//    tmp_msg.header = h;
//    planes_pub_.publish(tmp_msg);
}

void LidarOdometry::publishOpt(std_msgs::Header h) {
    // 发布tf
    static tf::TransformBroadcaster tb;
    tf::Transform trans;
    Eigen::Vector3d t(current_pose_opt_.translation().matrix());
    trans.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    Eigen::Quaterniond q(current_pose_opt_.rotation());
    tf::Quaternion tf_q;
    tf_q.setW(q.w());
    tf_q.setX(q.x());
    tf_q.setY(q.y());
    tf_q.setZ(q.z());
    trans.setRotation(tf_q);
    tb.sendTransform(tf::StampedTransform(trans, h.stamp, "map", "velodyne"));
    // 发布odo
    nav_msgs::Odometry odo_msg;
    odo_msg.header.stamp = h.stamp;
    odo_msg.header.frame_id = "map";
    odo_msg.child_frame_id = "velodyne";
    odo_msg.pose.pose.position.x = t.x();
    odo_msg.pose.pose.position.y = t.y();
    odo_msg.pose.pose.position.z = t.z();
    odo_msg.pose.pose.orientation.w = q.w();
    odo_msg.pose.pose.orientation.x = q.x();
    odo_msg.pose.pose.orientation.y = q.y();
    odo_msg.pose.pose.orientation.z = q.z();
    opt_odo_pub_.publish(odo_msg);
    // 发布路径
    static nav_msgs::Path path_opt;
    path_opt.header.frame_id = "map";
    geometry_msgs::PoseStamped p;
    p.header.stamp = h.stamp;
    p.header.frame_id = "map";
    p.pose.position.x = t.x();
    p.pose.position.y = t.y();
    p.pose.position.z = t.z();
    p.pose.orientation.w = q.w();
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    path_opt.poses.emplace_back(p);
    opt_odo_path_pub_.publish(path_opt);
    // 发布点云
    sensor_msgs::PointCloud2 tmp_msg;
    pcl::toROSMsg(*last_corners_, tmp_msg);
    tmp_msg.header = h;
    corners_pub_.publish(tmp_msg);
    pcl::toROSMsg(*last_planes_, tmp_msg);
    tmp_msg.header = h;
    planes_pub_.publish(tmp_msg);
}

bool LidarOdometry::work(const DataGroupPtr& data_group, const std::vector<KeyFramePtr>& key_frames, KeyFramePtr& new_frame) {
//    LOG(INFO) << "odo work...";
//    ROS_INFO("Data center add in odo: %p", DataCenter::Instance());
    if(!is_initialized){
        *last_corners_ = *data_group->less_corner_cloud;
        *last_planes_ = *data_group->less_plane_cloud;
        last_scan2scan_pose_ = current_pose_ = current_pose_opt_ = odo_drift_ = last_history_pose_ = Eigen::Affine3d::Identity();
        accumulate_dis_ = 0.0;
//        key_frames.emplace_back(std::make_shared<KeyFrame>(current_pose_opt_, accumulate_dis_, data_group->less_corner_cloud, data_group->less_plane_cloud));
        new_frame = std::make_shared<KeyFrame>(key_frames.size(), current_pose_, accumulate_dis_, current_pose_opt_, data_group->less_corner_cloud, data_group->less_plane_cloud);
        is_initialized = true;
        LOG(INFO) << "Odo is initialized...";
        return true;
    }
    LOG(INFO) << "Start odo process-----------------";
    Timer odo_work_timer("-------------------------odo work");
//    LOG(INFO) << "Address: " << data_group->corner_cloud.get() << ", " << data_group->plane_cloud.get();
    PointCloudXYZIPtr current_corners = data_group->corner_cloud;
    PointCloudXYZIPtr current_planes = data_group->plane_cloud;
//    LOG(INFO) << "Got data from data_group...";
    // Scan to scan register
    // TODO： 使用上一步的结果作为初始值，会更快，但是如果上一步不太好的话会加快积累
//    scan2scan_matcher_.reset();
//    LOG(INFO) << "Points size: " << current_corners->points.size() << ", " << current_planes->points.size() << ", "
//        << last_corners_->points.size() << ", " << last_planes_->points.size();
    Eigen::Affine3d init_pose = Eigen::Affine3d::Identity();
    if(use_const_velo_) init_pose = last_scan2scan_pose_;
    scan2scan_matcher_.align(last_corners_, last_planes_, current_corners, current_planes, init_pose);
    // Integrate pose
    Eigen::Affine3d delta_pose = scan2scan_matcher_.getTransform();
    current_pose_ = current_pose_ * delta_pose;
    {
        double x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(current_pose_, x, y, z, roll, pitch,yaw);
        LOG(INFO) << "Odo pose: " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw;
    }
    last_scan2scan_pose_ = delta_pose;
    // Prepare data for next loop
    if(deskew_){
        scan2scan_matcher_.cloudProjToEnd(data_group->less_corner_cloud);
        scan2scan_matcher_.cloudProjToEnd(data_group->less_plane_cloud);
    }
    last_corners_ = data_group->less_corner_cloud;
    last_planes_ = data_group->less_plane_cloud;

    publish(data_group->h);// 如果不重新发布原始点云，也就是说原始点云在tf发布之前就已经发布了，那么有可能导致插值出错点云乱跳

    //scan2map
    current_pose_opt_ = odo_drift_ * current_pose_;// correct odometry drift as init opt pose

    Eigen::Affine3d delta_history_pose = last_history_pose_.inverse() * current_pose_;
    bool is_key_frame = false;
    if(key_frames.size() < 5 || delta_history_pose.translation().norm() > 0.5 || Eigen::AngleAxisd(delta_history_pose.rotation()).angle() > 2 * ANG2RAD){
        // scan2map optimization
        is_key_frame = true;
        if(key_frames.size() >= 5){ // 帧数足够多才会优化
            // construct surrounding map
            // TODO: sliding window
            surrounding_corner_map_->clear();
            surrounding_plane_map_->clear();
//            LOG(INFO) << "Before Add: " << key_frames_.size() - 20;
            int num = key_frames.size();// 强制size_t转int，避免-负数溢出
            PointCloudXYZI tmp_cloud;
            for(int index = num - 1; index >= num - 20 && index >= 0; --index){ // 最近20帧
//                LOG(INFO) << "Add: " << (key_frames_.at(index)->corner_cloud_)->size();
                const KeyFrame& current_key = *(key_frames.at(index));
                tmp_cloud.clear();
                pcl::transformPointCloud(*(current_key.corner_cloud_), tmp_cloud, current_key.pose_);
                *surrounding_corner_map_ += tmp_cloud;
                tmp_cloud.clear();
                pcl::transformPointCloud(*(current_key.plane_cloud_), tmp_cloud, current_key.pose_);
                *surrounding_plane_map_ += tmp_cloud;
            }
            // 发布周围环境地图
            {
                if(surrounding_map_pub_.getNumSubscribers() > 0){
                    surrounding_map_->clear();
                    *surrounding_map_ += *surrounding_corner_map_;
                    *surrounding_map_ += *surrounding_plane_map_;
                    sensor_msgs::PointCloud2 cloud_msg;
                    pcl::toROSMsg(*surrounding_map_, cloud_msg);
                    cloud_msg.header.stamp = data_group->h.stamp;
                    cloud_msg.header.frame_id = "map";
                    surrounding_map_pub_.publish(cloud_msg);
                }
//                // debug
//                static int index = 0;
//                pcl::io::savePCDFileBinaryCompressed("/home/jin/Documents/lab_slam_ws/src/lab_slam/tmp/odo/submap_" + std::to_string(index) + ".pcd", *surrounding_map_);
//                surrounding_map_->clear();
//                PointCloudXYZI tmp;
//                pcl::transformPointCloud(*data_group->less_corner_cloud, tmp, current_pose_opt_);
//                *surrounding_map_ += tmp;
//                pcl::transformPointCloud(*data_group->less_plane_cloud, tmp, current_pose_opt_);
//                *surrounding_map_ += tmp;
//                pcl::io::savePCDFileBinaryCompressed("/home/jin/Documents/lab_slam_ws/src/lab_slam/tmp/odo/scan_" + std::to_string(index) + ".pcd", *surrounding_map_);
//                index++;
            }
            LOG(INFO) << "Surrounding map size: " << surrounding_corner_map_->size() << ", " << surrounding_plane_map_->size();
            // TODO: scan2map opt
//            scan2map_mather_.align(surrounding_corner_map_, surrounding_plane_map_, current_corners, current_planes, current_pose_opt_);
            scan2map_mather_.align(surrounding_corner_map_, surrounding_plane_map_, data_group->less_corner_cloud, data_group->less_plane_cloud, current_pose_opt_);
            current_pose_opt_ = scan2map_mather_.getTransform();
            odo_drift_ = current_pose_opt_ * current_pose_.inverse();// (world<-lidar)*(lidar<-odo)==(world<-odo)
        }else{
            LOG(INFO) << "Less than 5";
        }
        // save key frames
        accumulate_dis_ += delta_history_pose.translation().norm();
//        key_frames.emplace_back(std::make_shared<KeyFrame>(current_pose_opt_, accumulate_dis_, data_group->less_corner_cloud, data_group->less_plane_cloud));
        new_frame = std::make_shared<KeyFrame>(key_frames.size(), current_pose_, accumulate_dis_, current_pose_opt_, data_group->less_corner_cloud, data_group->less_plane_cloud);
        last_history_pose_ = current_pose_;// 用优化前的scan2scan位姿
        LOG(INFO) << "KeyFrame size: " << key_frames.size();
    }
    {
        double x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(current_pose_opt_, x, y, z, roll, pitch,yaw);
        LOG(INFO) << "Opt pose: " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw;
    }
    publishOpt(data_group->h);// 无论是不是关键帧
    if(is_key_frame){
        origin_cloud_repub_.publish(data_group->cloud_msg);
//        sensor_msgs::PointCloud2 tmp_cloud;
//        tmp_cloud = *data_group->cloud_msg;
//        tmp_cloud.header.frame_id = "odo";
//        origin_cloud_repub_.publish(tmp_cloud);
    }

    odo_work_timer.end();
    return is_key_frame;
}