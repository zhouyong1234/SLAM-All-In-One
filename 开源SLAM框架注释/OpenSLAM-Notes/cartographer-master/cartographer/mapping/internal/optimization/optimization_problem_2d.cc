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

#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/optimization/ceres_pose.h"
#include "cartographer/mapping/internal/optimization/cost_functions/landmark_cost_function_2d.h"
#include "cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_2d.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

using ::cartographer::mapping::optimization::CeresPose;
using LandmarkNode = ::cartographer::mapping::PoseGraphInterface::LandmarkNode;
using TrajectoryData =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryData;

// For fixed frame pose.
std::unique_ptr<transform::Rigid3d> Interpolate(
    const sensor::MapByTime<sensor::FixedFramePoseData>& map_by_time,
    const int trajectory_id, const common::Time time) {
  const auto it = map_by_time.lower_bound(trajectory_id, time);
  if (it == map_by_time.EndOfTrajectory(trajectory_id) ||
      !it->pose.has_value()) {
    return nullptr;
  }
  if (it == map_by_time.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid3d>(it->pose.value());
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  if (prev_it->pose.has_value()) {
    return absl::make_unique<transform::Rigid3d>(
        Interpolate(transform::TimestampedTransform{prev_it->time,
                                                    prev_it->pose.value()},
                    transform::TimestampedTransform{it->time, it->pose.value()},
                    time)
            .transform);
  }
  return nullptr;
}

// Converts a pose into the 3 optimization variable format used for Ceres:
// translation in x and y, followed by the rotation angle representing the
// orientation.
std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
  return {{pose.translation().x(), pose.translation().y(),
           pose.normalized_angle()}};
}

// Converts a pose as represented for Ceres back to an transform::Rigid2d pose.
transform::Rigid2d ToPose(const std::array<double, 3>& values) {
  return transform::Rigid2d({values[0], values[1]}, values[2]);
}

// Selects a trajectory node closest in time to the landmark observation and
// applies a relative transform from it.
transform::Rigid3d GetInitialLandmarkPose(
    const LandmarkNode::LandmarkObservation& observation,
    const NodeSpec2D& prev_node, const NodeSpec2D& next_node,
    const std::array<double, 3>& prev_node_pose,
    const std::array<double, 3>& next_node_pose) {
  const double interpolation_parameter =
      common::ToSeconds(observation.time - prev_node.time) /
      common::ToSeconds(next_node.time - prev_node.time);

  const std::tuple<std::array<double, 4>, std::array<double, 3>>
      rotation_and_translation =
          InterpolateNodes2D(prev_node_pose.data(), prev_node.gravity_alignment,
                             next_node_pose.data(), next_node.gravity_alignment,
                             interpolation_parameter);
  return transform::Rigid3d::FromArrays(std::get<0>(rotation_and_translation),
                                        std::get<1>(rotation_and_translation)) *
         observation.landmark_to_tracking_transform;
}

void AddLandmarkCostFunctions(
    const std::map<std::string, LandmarkNode>& landmark_nodes,
    const MapById<NodeId, NodeSpec2D>& node_data,
    MapById<NodeId, std::array<double, 3>>* C_nodes,
    std::map<std::string, CeresPose>* C_landmarks, ceres::Problem* problem,
    double huber_scale) {
  for (const auto& landmark_node : landmark_nodes) {
    for (const auto& observation : landmark_node.second.landmark_observations) {
      const std::string& landmark_id = landmark_node.first;
      const auto& begin_of_trajectory =
          node_data.BeginOfTrajectory(observation.trajectory_id);
      // The landmark observation was made before the trajectory was created.
      if (observation.time < begin_of_trajectory->data.time) {
        continue;
      }
      // Find the trajectory nodes before and after the landmark observation.
      auto next =
          node_data.lower_bound(observation.trajectory_id, observation.time);
      // The landmark observation was made, but the next trajectory node has
      // not been added yet.
      if (next == node_data.EndOfTrajectory(observation.trajectory_id)) {
        continue;
      }
      if (next == begin_of_trajectory) {
        next = std::next(next);
      }
      auto prev = std::prev(next);
      // Add parameter blocks for the landmark ID if they were not added before.
      std::array<double, 3>* prev_node_pose = &C_nodes->at(prev->id);
      std::array<double, 3>* next_node_pose = &C_nodes->at(next->id);
      if (!C_landmarks->count(landmark_id)) {
        const transform::Rigid3d starting_point =
            landmark_node.second.global_landmark_pose.has_value()
                ? landmark_node.second.global_landmark_pose.value()
                : GetInitialLandmarkPose(observation, prev->data, next->data,
                                         *prev_node_pose, *next_node_pose);
        C_landmarks->emplace(
            landmark_id,
            CeresPose(starting_point, nullptr /* translation_parametrization */,
                      absl::make_unique<ceres::QuaternionParameterization>(),
                      problem));
        // Set landmark constant if it is frozen.
        if (landmark_node.second.frozen) {
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).translation());
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).rotation());
        }
      }
      problem->AddResidualBlock(
          LandmarkCostFunction2D::CreateAutoDiffCostFunction(
              observation, prev->data, next->data),
          new ceres::HuberLoss(huber_scale), prev_node_pose->data(),
          next_node_pose->data(), C_landmarks->at(landmark_id).rotation(),
          C_landmarks->at(landmark_id).translation());
    }
  }
}

}  // namespace
// 构造函数，单纯复制配置参数
OptimizationProblem2D::OptimizationProblem2D(
    const proto::OptimizationProblemOptions& options)
    : options_(options) {}
// 析构函数，啥都没做
OptimizationProblem2D::~OptimizationProblem2D() {}

// 添加imu数据，在2d激光中没有添加imu数据到后端中
// 也就是说imu在2d情况下仅仅用于前端辅助快速匹配
void OptimizationProblem2D::AddImuData(const int trajectory_id,
                                       const sensor::ImuData& imu_data) {
  // IMU data is not used in 2D optimization, so we ignore this part of the
  // interface.
}

// 添加里程计数据 ---> odometry_data_
void OptimizationProblem2D::AddOdometryData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  odometry_data_.Append(trajectory_id, odometry_data);
}
// 添加固定帧数据 --> fixed_frame_pose_data_
void OptimizationProblem2D::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  fixed_frame_pose_data_.Append(trajectory_id, fixed_frame_pose_data);
}
// 添加轨迹节点 --> node_data_
void OptimizationProblem2D::AddTrajectoryNode(const int trajectory_id,
                                              const NodeSpec2D& node_data) {
  node_data_.Append(trajectory_id, node_data);
  trajectory_data_[trajectory_id];
}

// 设置轨迹节点 
void OptimizationProblem2D::SetTrajectoryData(
    int trajectory_id, const TrajectoryData& trajectory_data) {
  trajectory_data_[trajectory_id] = trajectory_data;
}

// 插入轨迹节点
void OptimizationProblem2D::InsertTrajectoryNode(const NodeId& node_id,
                                                 const NodeSpec2D& node_data) {
  node_data_.Insert(node_id, node_data);
  trajectory_data_[node_id.trajectory_id];
}
// 微调轨迹节点
void OptimizationProblem2D::TrimTrajectoryNode(const NodeId& node_id) {
  // 所有节点均进行微调
  empty_imu_data_.Trim(node_data_, node_id);
  odometry_data_.Trim(node_data_, node_id);
  fixed_frame_pose_data_.Trim(node_data_, node_id);
  node_data_.Trim(node_id);
  if (node_data_.SizeOfTrajectoryOrZero(node_id.trajectory_id) == 0) {
    trajectory_data_.erase(node_id.trajectory_id);
  }
}
// 添加子地图 
void OptimizationProblem2D::AddSubmap(
    const int trajectory_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Append(trajectory_id, SubmapSpec2D{global_submap_pose});
}
// 插入子地图
void OptimizationProblem2D::InsertSubmap(
    const SubmapId& submap_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Insert(submap_id, SubmapSpec2D{global_submap_pose});
}
// 微调子地图
void OptimizationProblem2D::TrimSubmap(const SubmapId& submap_id) {
  submap_data_.Trim(submap_id);
}
// 设置最大迭代次数
void OptimizationProblem2D::SetMaxNumIterations(
    const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}
// 求解优化问题
// constraints  约束项
// trajectories_state  轨迹状态  
// landmark_nodes      路标点节点
void OptimizationProblem2D::Solve(
    const std::vector<Constraint>& constraints,
    const std::map<int, PoseGraphInterface::TrajectoryState>&
        trajectories_state,
    const std::map<std::string, LandmarkNode>& landmark_nodes) {
  // 没有需要优化的节点
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }
  // 当前轨迹为冻结状态，则不进行优化
  std::set<int> frozen_trajectories;
  for (const auto& it : trajectories_state) {
    if (it.second == PoseGraphInterface::TrajectoryState::FROZEN) {
      frozen_trajectories.insert(it.first);
    }
  }
  // 构建 ceres 优化问题
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // Set the starting point.
  // TODO(hrapp): Move ceres data into SubmapSpec.
  MapById<SubmapId, std::array<double, 3>> C_submaps;
  MapById<NodeId, std::array<double, 3>> C_nodes;
  std::map<std::string, CeresPose> C_landmarks;
  bool first_submap = true;
  // 遍历所有子地图
  for (const auto& submap_id_data : submap_data_) {
  	// 当前子地图是否为冻结状态
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
	// 插入到C_submaps 
	C_submaps.Insert(submap_id_data.id,
                     FromPose(submap_id_data.data.global_pose));
	// 添加参数块 ---> 子地图的位置 
    problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);
	// 第一张地图和被冻结的地图不会更新其位置
    if (first_submap || frozen) {
      first_submap = false;
      // Fix the pose of the first submap or all submaps of a frozen
      // trajectory.
      problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
    }
  }
  // 遍历所有的节点
  for (const auto& node_id_data : node_data_) {
  	// 当前节点状态是否为冻结
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
	// 插入到C_nodes中
    C_nodes.Insert(node_id_data.id, FromPose(node_id_data.data.global_pose_2d));
    // 添加参数块
	problem.AddParameterBlock(C_nodes.at(node_id_data.id).data(), 3);
	// 冻结状态不会更新
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).data());
    }
  }
  // Add cost functions for intra- and inter-submap constraints.
  // 遍历所有约束
  for (const Constraint& constraint : constraints) {
  	// 添加残差
  	// 1 如果是回环检测得到的约束，则添加huber鲁棒核函数，避免错误匹配带来过大误差
  	// 2 在对应的C_submaps和C_nodes中添加约束
    problem.AddResidualBlock(
        CreateAutoDiffSpaCostFunction(constraint.pose),
        // Loop closure constraints should have a loss function.
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr,
        C_submaps.at(constraint.submap_id).data(),
        C_nodes.at(constraint.node_id).data());
  }
  // 添加路标点的误差函数
  // Add cost functions for landmarks.
  AddLandmarkCostFunctions(landmark_nodes, node_data_, &C_nodes, &C_landmarks,
                           &problem, options_.huber_scale());
  // Add penalties for violating odometry or changes between consecutive nodes
  // if odometry is not available.
  // 遍历所有节点，添加里程计的误差函数
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    if (frozen_trajectories.count(trajectory_id) != 0) {
      node_it = trajectory_end;
      continue;
    }

    auto prev_node_it = node_it;
    for (++node_it; node_it != trajectory_end; ++node_it) {
      const NodeId first_node_id = prev_node_it->id;
      const NodeSpec2D& first_node_data = prev_node_it->data;
      prev_node_it = node_it;
      const NodeId second_node_id = node_it->id;
      const NodeSpec2D& second_node_data = node_it->data;

      if (second_node_id.node_index != first_node_id.node_index + 1) {
        continue;
      }

      // Add a relative pose constraint based on the odometry (if available).
      std::unique_ptr<transform::Rigid3d> relative_odometry =
          CalculateOdometryBetweenNodes(trajectory_id, first_node_data,
                                        second_node_data);
      if (relative_odometry != nullptr) {
        problem.AddResidualBlock(
            CreateAutoDiffSpaCostFunction(Constraint::Pose{
                *relative_odometry, options_.odometry_translation_weight(),
                options_.odometry_rotation_weight()}),
            nullptr /* loss function */, C_nodes.at(first_node_id).data(),
            C_nodes.at(second_node_id).data());
      }

      // Add a relative pose constraint based on consecutive local SLAM poses.
      const transform::Rigid3d relative_local_slam_pose =
          transform::Embed3D(first_node_data.local_pose_2d.inverse() *
                             second_node_data.local_pose_2d);
      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(
              Constraint::Pose{relative_local_slam_pose,
                               options_.local_slam_pose_translation_weight(),
                               options_.local_slam_pose_rotation_weight()}),
          nullptr /* loss function */, C_nodes.at(first_node_id).data(),
          C_nodes.at(second_node_id).data());
    }
  }

  std::map<int, std::array<double, 3>> C_fixed_frames;
  // 遍历所有节点，添加fixed_frame_pose的约束
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    if (!fixed_frame_pose_data_.HasTrajectory(trajectory_id)) {
      node_it = trajectory_end;
      continue;
    }

    const TrajectoryData& trajectory_data = trajectory_data_.at(trajectory_id);
    bool fixed_frame_pose_initialized = false;
    for (; node_it != trajectory_end; ++node_it) {
      const NodeId node_id = node_it->id;
      const NodeSpec2D& node_data = node_it->data;

      const std::unique_ptr<transform::Rigid3d> fixed_frame_pose =
          Interpolate(fixed_frame_pose_data_, trajectory_id, node_data.time);
      if (fixed_frame_pose == nullptr) {
        continue;
      }

      const Constraint::Pose constraint_pose{
          *fixed_frame_pose, options_.fixed_frame_pose_translation_weight(),
          options_.fixed_frame_pose_rotation_weight()};

      if (!fixed_frame_pose_initialized) {
        transform::Rigid2d fixed_frame_pose_in_map;
        if (trajectory_data.fixed_frame_origin_in_map.has_value()) {
          fixed_frame_pose_in_map = transform::Project2D(
              trajectory_data.fixed_frame_origin_in_map.value());
        } else {
          fixed_frame_pose_in_map =
              node_data.global_pose_2d *
              transform::Project2D(constraint_pose.zbar_ij).inverse();
        }

        C_fixed_frames.emplace(trajectory_id,
                               FromPose(fixed_frame_pose_in_map));
        fixed_frame_pose_initialized = true;
      }

      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(constraint_pose),
          options_.fixed_frame_pose_use_tolerant_loss()
              ? new ceres::TolerantLoss(
                    options_.fixed_frame_pose_tolerant_loss_param_a(),
                    options_.fixed_frame_pose_tolerant_loss_param_b())
              : nullptr,
          C_fixed_frames.at(trajectory_id).data(), C_nodes.at(node_id).data());
    }
  }

  // Solve.
  // 求解问题
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);
  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
  }

  // Store the result.
  // 保存结果
  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).global_pose =
        ToPose(C_submap_id_data.data);
  }
  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).global_pose_2d =
        ToPose(C_node_id_data.data);
  }
  for (const auto& C_fixed_frame : C_fixed_frames) {
    trajectory_data_.at(C_fixed_frame.first).fixed_frame_origin_in_map =
        transform::Embed3D(ToPose(C_fixed_frame.second));
  }
  for (const auto& C_landmark : C_landmarks) {
    landmark_data_[C_landmark.first] = C_landmark.second.ToRigid();
  }
}

std::unique_ptr<transform::Rigid3d> OptimizationProblem2D::InterpolateOdometry(
    const int trajectory_id, const common::Time time) const {
  const auto it = odometry_data_.lower_bound(trajectory_id, time);
  if (it == odometry_data_.EndOfTrajectory(trajectory_id)) {
    return nullptr;
  }
  if (it == odometry_data_.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid3d>(it->pose);
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  return absl::make_unique<transform::Rigid3d>(
      Interpolate(transform::TimestampedTransform{prev_it->time, prev_it->pose},
                  transform::TimestampedTransform{it->time, it->pose}, time)
          .transform);
}

std::unique_ptr<transform::Rigid3d>
OptimizationProblem2D::CalculateOdometryBetweenNodes(
    const int trajectory_id, const NodeSpec2D& first_node_data,
    const NodeSpec2D& second_node_data) const {
  if (odometry_data_.HasTrajectory(trajectory_id)) {
    const std::unique_ptr<transform::Rigid3d> first_node_odometry =
        InterpolateOdometry(trajectory_id, first_node_data.time);
    const std::unique_ptr<transform::Rigid3d> second_node_odometry =
        InterpolateOdometry(trajectory_id, second_node_data.time);
    if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
      transform::Rigid3d relative_odometry =
          transform::Rigid3d::Rotation(first_node_data.gravity_alignment) *
          first_node_odometry->inverse() * (*second_node_odometry) *
          transform::Rigid3d::Rotation(
              second_node_data.gravity_alignment.inverse());
      return absl::make_unique<transform::Rigid3d>(relative_odometry);
    }
  }
  return nullptr;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
