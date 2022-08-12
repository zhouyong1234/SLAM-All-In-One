//
// Created by jin on 2021/6/1.
//

#ifndef LAB_SLAM_LAB_SLAM_H
#define LAB_SLAM_LAB_SLAM_H

#include "utility.hpp"
#include "pre_processor.h"
#include "lidar_odometry.h"
#include "loop_detection.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <pcl_conversions/pcl_conversions.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;
using namespace gtsam::symbol_shorthand;
class LabSLAM{
public:
    LabSLAM();
    void msgCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void preprocessWork();
    void lidarOdoWork();
    void loopClosure();
    void publishGlobalMap();
private:
    // std::string topic_name_ = std::string("/points_raw");// velodyne_points
   std::string topic_name_ = std::string("/velodyne_points");// velodyne_points
    ros::Subscriber cloud_msg_sub_;
    PreProcessor pre_processor_;
    LidarOdometry lidar_odo_;
    std::deque<sensor_msgs::PointCloud2::ConstPtr> velodyne_msgs_;
    std::mutex velodyne_msg_mutex_;
    std::condition_variable msg_condit_var_;
    std::deque<DataGroupPtr> data_;
    std::mutex data_mutex_;
    std::condition_variable data_condit_var_;
    std::vector<KeyFramePtr> key_frames_;
    std::mutex key_frame_mutex_;

    bool new_key_frame_added_ = false;
    LoopDetection loop_detection_;
    std::mutex closure_mutex_;
    std::condition_variable closure_var_;
    std::vector<LoopClosurePair> closure_pairs_;
    ros::Publisher loop_closure_pub_;

    std::shared_ptr<gtsam::ISAM2> optimizer_ = nullptr;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values values_;
    int new_node_index_ = 0;

    ros::Publisher global_map_pub_;
    bool publish_global_map_flag_ = false;
    std::mutex publish_global_map_mutex_;
    std::condition_variable publish_global_map_var_;
};
#endif //LAB_SLAM_LAB_SLAM_H
