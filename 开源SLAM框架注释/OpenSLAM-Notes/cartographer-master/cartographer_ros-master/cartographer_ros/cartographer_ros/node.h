/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "cartographer_ros_msgs/ReadMetrics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/WriteState.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"

namespace cartographer_ros {

// Wires up ROS topics to SLAM.
// 将ROS消息数据添加到SLAM系统中
// 相当于这个类就是SLAM系统到ROS的一个接口类
class Node {
 public:
  Node(const NodeOptions& node_options,
       std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
       tf2_ros::Buffer* tf_buffer, bool collect_metrics);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  // Finishes all yet active trajectories.
  // 完成了所有轨迹
  void FinishAllTrajectories();
  // Finishes a single given trajectory. Returns false if the trajectory did not
  // exist or was already finished.
  // 设置某一条轨迹完成 
  bool FinishTrajectory(int trajectory_id);

  // Runs final optimization. All trajectories have to be finished when calling.
  // 当所有轨迹结束了，运行最终的优化程序 
  void RunFinalOptimization();

  // Starts the first trajectory with the default topics.
  // 开始第一条轨迹，使用默认的消息 
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);
  
  // Returns unique SensorIds for multiple input bag files based on
  // their TrajectoryOptions.
  // 'SensorId::id' is the expected ROS topic name.
  std::vector<
      std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
  ComputeDefaultSensorIdsForMultipleBags(
      const std::vector<TrajectoryOptions>& bags_options) const;

  // Adds a trajectory for offline processing, i.e. not listening to topics.
  // 添加离线的轨迹 
  int AddOfflineTrajectory(
      const std::set<
          cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& options);

  // The following functions handle adding sensor data to a trajectory.
  // 往轨迹中添加里程计信息
  void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                             const nav_msgs::Odometry::ConstPtr& msg);
  // 添加GPS信息 
  void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::NavSatFix::ConstPtr& msg);
  // 添加路标点信息，如自定义的二维码
  void HandleLandmarkMessage(
      int trajectory_id, const std::string& sensor_id,
      const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);
  // 添加IMU数据
  void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                        const sensor_msgs::Imu::ConstPtr& msg);
  // 添加激光雷达数据                       
  void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  // 减价激光雷达Echo类型数据，多一个强度数据 
  void HandleMultiEchoLaserScanMessage(
      int trajectory_id, const std::string& sensor_id,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  // 添加点云数据   
  void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::PointCloud2::ConstPtr& msg);

  // Serializes the complete Node state.
  // 序列化当前的状态，即将当前所有信息保存到文件中 
  void SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);
  // 读取之间的构建的地图信息 .pbstream 文件 
  // Loads a serialized SLAM state from a .pbstream file.
  void LoadState(const std::string& state_filename, bool load_frozen_state);
  // ROS handle 
  ::ros::NodeHandle* node_handle();

 private:
 // 私有的函数和变量
  struct Subscriber {
    ::ros::Subscriber subscriber;

    // ::ros::Subscriber::getTopic() does not necessarily return the same
    // std::string
    // it was given in its constructor. Since we rely on the topic name as the
    // unique identifier of a subscriber, we remember it ourselves.
    std::string topic;
  };

  bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleTrajectoryQuery(
      ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
      ::cartographer_ros_msgs::TrajectoryQuery::Response& response);
  bool HandleStartTrajectory(
      cartographer_ros_msgs::StartTrajectory::Request& request,
      cartographer_ros_msgs::StartTrajectory::Response& response);
  bool HandleFinishTrajectory(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);
  bool HandleWriteState(cartographer_ros_msgs::WriteState::Request& request,
                        cartographer_ros_msgs::WriteState::Response& response);
  bool HandleGetTrajectoryStates(
      ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
      ::cartographer_ros_msgs::GetTrajectoryStates::Response& response);
  bool HandleReadMetrics(
      cartographer_ros_msgs::ReadMetrics::Request& request,
      cartographer_ros_msgs::ReadMetrics::Response& response);

  // Returns the set of SensorIds expected for a trajectory.
  // 'SensorId::id' is the expected ROS topic name.
  std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
  ComputeExpectedSensorIds(const TrajectoryOptions& options) const;
  int AddTrajectory(const TrajectoryOptions& options);
  void LaunchSubscribers(const TrajectoryOptions& options, int trajectory_id);
  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);
  void PublishLocalTrajectoryData(const ::ros::TimerEvent& timer_event);
  void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
  void PublishLandmarkPosesList(const ::ros::WallTimerEvent& timer_event);
  void PublishConstraintList(const ::ros::WallTimerEvent& timer_event);
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const TrajectoryOptions& options);
  cartographer_ros_msgs::StatusResponse FinishTrajectoryUnderLock(
      int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void MaybeWarnAboutTopicMismatch(const ::ros::WallTimerEvent&);

  // Helper function for service handlers that need to check trajectory states.
  cartographer_ros_msgs::StatusResponse TrajectoryStateToStatus(
      int trajectory_id,
      const std::set<
          cartographer::mapping::PoseGraphInterface::TrajectoryState>&
          valid_states);
  const NodeOptions node_options_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  absl::Mutex mutex_;
  std::unique_ptr<cartographer_ros::metrics::FamilyFactory> metrics_registry_;
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

  ::ros::NodeHandle node_handle_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::Publisher trajectory_node_list_publisher_;
  ::ros::Publisher landmark_poses_list_publisher_;
  ::ros::Publisher constraint_list_publisher_;
  ::ros::Publisher tracked_pose_publisher_;
  // These ros::ServiceServers need to live for the lifetime of the node.
  std::vector<::ros::ServiceServer> service_servers_;
  ::ros::Publisher scan_matched_point_cloud_publisher_;
  
  // 传感器采样结构体
  struct TrajectorySensorSamplers {
	// 配置参数  
    TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                             const double odometry_sampling_ratio,
                             const double fixed_frame_pose_sampling_ratio,
                             const double imu_sampling_ratio,
                             const double landmark_sampling_ratio)
        : rangefinder_sampler(rangefinder_sampling_ratio),
          odometry_sampler(odometry_sampling_ratio),
          fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
          imu_sampler(imu_sampling_ratio),
          landmark_sampler(landmark_sampling_ratio) {}
    //在应从数据流中抽取样本以选择数据的均匀分布部分时发出信号。
    ::cartographer::common::FixedRatioSampler rangefinder_sampler;
    ::cartographer::common::FixedRatioSampler odometry_sampler;
    ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
    ::cartographer::common::FixedRatioSampler imu_sampler;
    ::cartographer::common::FixedRatioSampler landmark_sampler;
  };

  // These are keyed with 'trajectory_id'.
  // 保持一定的姿势持续时间以估计线性和角速度。
  // 使用速度推断运动。使用IMU和/或里程计数据（如果有）来改善推断。
  std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
  std::map<int, ::ros::Time> last_published_tf_stamps_;
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
  std::unordered_map<int, std::vector<Subscriber>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_set<int> trajectories_scheduled_for_finish_;

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;

  // The timer for publishing local trajectory data (i.e. pose transforms and
  // range data point clouds) is a regular timer which is not triggered when
  // simulation time is standing still. This prevents overflowing the transform
  // listener buffer by publishing the same transforms over and over again.
  ::ros::Timer publish_local_trajectory_data_timer_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
