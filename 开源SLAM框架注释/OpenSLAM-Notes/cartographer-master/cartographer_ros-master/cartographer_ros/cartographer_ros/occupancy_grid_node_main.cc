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

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/synchronization/mutex.h"
#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "gflags/gflags.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");
DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
              "Name of the topic on which the occupancy grid is published.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node {
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  void DrawAndPublish(const ::ros::WallTimerEvent& timer_event);

  ::ros::NodeHandle node_handle_;
  const double resolution_;

  absl::Mutex mutex_;
  ::ros::ServiceClient client_ GUARDED_BY(mutex_);
  ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
  ::ros::Publisher occupancy_grid_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  ::ros::WallTimer occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  ros::Time last_timestamp_;
};

// node 类构造函数 
// 这个构造函数就写的b格很高
Node::Node(const double resolution, const double publish_period_sec)
    : resolution_(resolution),
      client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
          kSubmapQueryServiceName)),
      // 构造订阅子地图的变量 submap_list_subscriber_ 
      // 消息回调函数：HandleSubmapList
      submap_list_subscriber_(node_handle_.subscribe(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize,
          boost::function<void(
              const cartographer_ros_msgs::SubmapList::ConstPtr&)>(
              [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
                HandleSubmapList(msg);
              }))),
      // 构造栅格地图发布变量 occupancy_grid_publisher_         
      occupancy_grid_publisher_(
          node_handle_.advertise<::nav_msgs::OccupancyGrid>(
              FLAGS_occupancy_grid_topic, kLatestOnlyPublisherQueueSize,
              true /* latched */)),
      // 新建一个定时器，调用函数 DrawAndPublish 发布栅格地图信息
      occupancy_grid_publisher_timer_(
          node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec),
                                       &Node::DrawAndPublish, this)) {}
// 子地图数据接收回调函数
void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  absl::MutexLock locker(&mutex_);

  // We do not do any work if nobody listens.
  if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
    return;
  }

  // Keep track of submap IDs that don't appear in the message anymore.
  // 把之前的所有子地图放到待删除列表中
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }
  // 遍历所有接收到的子地图
  for (const auto& submap_msg : msg->submap) {
    // 构造id 
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    // 这个对应的id的子地图就不删除 
    submap_ids_to_delete.erase(id);
    // 如果这个子地图不更新的话，直接跳过
    if ((submap_msg.is_frozen && !FLAGS_include_frozen_submaps) ||
        (!submap_msg.is_frozen && !FLAGS_include_unfrozen_submaps)) {
      continue;
    }
    // 当前子地图的应用
    SubmapSlice& submap_slice = submap_slices_[id];
    // 更新子地图数据 
    submap_slice.pose = ToRigid3d(submap_msg.pose);
    submap_slice.metadata_version = submap_msg.submap_version;
    // 如果不是新的子地图就可以直接结束了
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version) {
      continue;
    }
    // 新的子地图处理
    // 调用函数 FetchSubmapTextures 获取子地图的数据 
    auto fetched_textures =
        ::cartographer_ros::FetchSubmapTextures(id, &client_);
    if (fetched_textures == nullptr) {
      continue;
    }
    // 
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    // 子地图数据获得 
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }

  // Delete all submaps that didn't appear in the message.
  // 删除掉所有没有再出现的子地图
  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

// 定时调用发布占用栅格地图
// 调用函数
// 1、PaintSubmapSlices
// 2、CreateOccupancyGridMsg
void Node::DrawAndPublish(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock locker(&mutex_);
  // 确保有数据 
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }
  // 绘制子地图,子地图数据 submap_slices_ 
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  // 创建占用栅格地图信息 
  std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  // 发布占用栅格信息 
  occupancy_grid_publisher_.publish(*msg_ptr);
}

}  // namespace
}  // namespace cartographer_ros

// 占用栅格地图节点
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  // 构造node节点 
  ::cartographer_ros::Node node(FLAGS_resolution, FLAGS_publish_period_sec);

  ::ros::spin();
  ::ros::shutdown();
}
