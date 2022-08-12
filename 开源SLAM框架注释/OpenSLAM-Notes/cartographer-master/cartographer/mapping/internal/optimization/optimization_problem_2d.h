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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_

#include <array>
#include <deque>
#include <map>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/pose_graph/optimization_problem_options.pb.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/map_by_time.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/timestamped_transform.h"

namespace cartographer {
namespace mapping {
namespace optimization {

struct NodeSpec2D {
  common::Time time;
  transform::Rigid2d local_pose_2d;
  transform::Rigid2d global_pose_2d;
  Eigen::Quaterniond gravity_alignment;
};

struct SubmapSpec2D {
  transform::Rigid2d global_pose;
};

// 2d位姿图优化问题
class OptimizationProblem2D
    : public OptimizationProblemInterface<NodeSpec2D, SubmapSpec2D,
                                          transform::Rigid2d> {
 public:
  // 构造析构函数	
  explicit OptimizationProblem2D(
      const optimization::proto::OptimizationProblemOptions& options);
  ~OptimizationProblem2D();
  // 禁止复制构造函数
  OptimizationProblem2D(const OptimizationProblem2D&) = delete;
  OptimizationProblem2D& operator=(const OptimizationProblem2D&) = delete;
  // 添加imu数据
  void AddImuData(int trajectory_id, const sensor::ImuData& imu_data) override;
  // 添加里程计数据
  void AddOdometryData(int trajectory_id,
                       const sensor::OdometryData& odometry_data) override;
  // 添加轨迹节点
  void AddTrajectoryNode(int trajectory_id,
                         const NodeSpec2D& node_data) override;
  // 插入轨迹节点
  void InsertTrajectoryNode(const NodeId& node_id,
                            const NodeSpec2D& node_data) override;
  // 调整节点
  void TrimTrajectoryNode(const NodeId& node_id) override;
  // 添加子地图
  void AddSubmap(int trajectory_id,
                 const transform::Rigid2d& global_submap_pose) override;
  // 插入子地图
  void InsertSubmap(const SubmapId& submap_id,
                    const transform::Rigid2d& global_submap_pose) override;
  // 调整子地图
  void TrimSubmap(const SubmapId& submap_id) override;
  // 设置最大迭代次数
  void SetMaxNumIterations(int32 max_num_iterations) override;
  // 求解优化问题
  void Solve(
      const std::vector<Constraint>& constraints,
      const std::map<int, PoseGraphInterface::TrajectoryState>&
          trajectories_state,
      const std::map<std::string, LandmarkNode>& landmark_nodes) override;
  // 获取 node data 
  const MapById<NodeId, NodeSpec2D>& node_data() const override {
    return node_data_;
  }
  // 获取 submap data 
  const MapById<SubmapId, SubmapSpec2D>& submap_data() const override {
    return submap_data_;
  }
  // 获取 landmark data
  const std::map<std::string, transform::Rigid3d>& landmark_data()
      const override {
    return landmark_data_;
  }
  // 获取 imu data
  const sensor::MapByTime<sensor::ImuData>& imu_data() const override {
    return empty_imu_data_;
  }
  // 获取里程计数据
  const sensor::MapByTime<sensor::OdometryData>& odometry_data()
      const override {
    return odometry_data_;
  }
  // 添加固定帧数据
  void AddFixedFramePoseData(
      int trajectory_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data);
  // 设置轨迹数据
  void SetTrajectoryData(
      int trajectory_id,
      const PoseGraphInterface::TrajectoryData& trajectory_data);
  const sensor::MapByTime<sensor::FixedFramePoseData>& fixed_frame_pose_data()
      const {
    return fixed_frame_pose_data_;
  }
  // 获取轨迹数据 
  const std::map<int, PoseGraphInterface::TrajectoryData>& trajectory_data()
      const {
    return trajectory_data_;
  }

 private:
  std::unique_ptr<transform::Rigid3d> InterpolateOdometry(
      int trajectory_id, common::Time time) const;
  // Computes the relative pose between two nodes based on odometry data.
  std::unique_ptr<transform::Rigid3d> CalculateOdometryBetweenNodes(
      int trajectory_id, const NodeSpec2D& first_node_data,
      const NodeSpec2D& second_node_data) const;

  optimization::proto::OptimizationProblemOptions options_;
  MapById<NodeId, NodeSpec2D> node_data_;
  MapById<SubmapId, SubmapSpec2D> submap_data_;
  std::map<std::string, transform::Rigid3d> landmark_data_;
  sensor::MapByTime<sensor::ImuData> empty_imu_data_;
  sensor::MapByTime<sensor::OdometryData> odometry_data_;
  sensor::MapByTime<sensor::FixedFramePoseData> fixed_frame_pose_data_;
  std::map<int, PoseGraphInterface::TrajectoryData> trajectory_data_;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_2D_H_
