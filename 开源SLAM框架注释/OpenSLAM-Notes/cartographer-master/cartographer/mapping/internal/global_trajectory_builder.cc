/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/global_trajectory_builder.h"

#include <memory>

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/metrics/family_factory.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

static auto* kLocalSlamMatchingResults = metrics::Counter::Null();
static auto* kLocalSlamInsertionResults = metrics::Counter::Null();


// 全局规矩规划类，在collated_trajectory_builder 类中使用

template <typename LocalTrajectoryBuilder, typename PoseGraph>
class GlobalTrajectoryBuilder : public mapping::TrajectoryBuilderInterface {
 public:
  // Passing a 'nullptr' for 'local_trajectory_builder' is acceptable, but no
  // 'TimedPointCloudData' may be added in that case.
  // 由局部轨迹生成和位姿图 构造
  GlobalTrajectoryBuilder(
      std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder,
      const int trajectory_id, PoseGraph* const pose_graph,
      const LocalSlamResultCallback& local_slam_result_callback,
      const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter)
      : trajectory_id_(trajectory_id),
        pose_graph_(pose_graph),
        local_trajectory_builder_(std::move(local_trajectory_builder)),
        local_slam_result_callback_(local_slam_result_callback),
        pose_graph_odometry_motion_filter_(pose_graph_odometry_motion_filter) {}
  ~GlobalTrajectoryBuilder() override {}
  // 禁止复制函数
  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;
  // 添加传感器数据 --- timed_point_cloud_data
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override 
  {
    // 确保当前局部轨迹规划非空
    CHECK(local_trajectory_builder_)
        << "Cannot add TimedPointCloudData without a LocalTrajectoryBuilder.";
    // 添加到局部轨迹规划器中，返回跟踪状态
	std::unique_ptr<typename LocalTrajectoryBuilder::MatchingResult>
        matching_result = local_trajectory_builder_->AddRangeData(
            sensor_id, timed_point_cloud_data);
	// 当前帧信息已经被完全累计
    if (matching_result == nullptr) {
      // The range data has not been fully accumulated yet.
      return;
    }
	// 
    kLocalSlamMatchingResults->Increment();
	// 插入结果
    std::unique_ptr<InsertionResult> insertion_result;
    if (matching_result->insertion_result != nullptr)
	{
      kLocalSlamInsertionResults->Increment();
	  // 添加到位姿图中
      auto node_id = pose_graph_->AddNode(
          matching_result->insertion_result->constant_data, trajectory_id_,
          matching_result->insertion_result->insertion_submaps);
	  
      CHECK_EQ(node_id.trajectory_id, trajectory_id_);
      // 插入结果 
	  insertion_result = absl::make_unique<InsertionResult>(InsertionResult{
          node_id, matching_result->insertion_result->constant_data,
          std::vector<std::shared_ptr<const Submap>>(
              matching_result->insertion_result->insertion_submaps.begin(),
              matching_result->insertion_result->insertion_submaps.end())});
    }
    if (local_slam_result_callback_) {
	  // 调用局部SLAM回调函数	
      local_slam_result_callback_(
          trajectory_id_, matching_result->time, matching_result->local_pose,
          std::move(matching_result->range_data_in_local),
          std::move(insertion_result));
    }
  }
  // 添加传感器数据 --- IMU
  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override
  {
    // 添加到局部路径规划器中
    if (local_trajectory_builder_)
	{
      local_trajectory_builder_->AddImuData(imu_data);
    }
	// 添加到位姿图中
    pose_graph_->AddImuData(trajectory_id_, imu_data);
  }
  // 添加传感器数据 --- odometry_data 
  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override
  {
    // 判断里程计是否有效
    CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
	// 添加到局部规划中
    if (local_trajectory_builder_) {
      local_trajectory_builder_->AddOdometryData(odometry_data);
    }
    // TODO(MichaelGrupp): Instead of having an optional filter on this level,
    // odometry could be marginalized between nodes in the pose graph.
    // Related issue: cartographer-project/cartographer/#1768
    if (pose_graph_odometry_motion_filter_.has_value() &&
        pose_graph_odometry_motion_filter_.value().IsSimilar(
            odometry_data.time, odometry_data.pose)) {
      return;
    }
	// 添加到位姿图中
    pose_graph_->AddOdometryData(trajectory_id_, odometry_data);
  }
  // 添加传感器信息 --- fixed_frame_pose 
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) override
  {
    // 固定帧数据可用
    if (fixed_frame_pose.pose.has_value()) {
      CHECK(fixed_frame_pose.pose.value().IsValid())
          << fixed_frame_pose.pose.value();
    }
	// 不添加到前端，直接添加到位姿图优化中
    pose_graph_->AddFixedFramePoseData(trajectory_id_, fixed_frame_pose);
  }
  // 添加传感器信息 --- landmark 
  void AddSensorData(const std::string& sensor_id,
                     const sensor::LandmarkData& landmark_data) override 
  {
    // 同样添加直接添加到位姿图优化中，而不是前端
    pose_graph_->AddLandmarkData(trajectory_id_, landmark_data);
  }

  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override {
    CHECK(!local_trajectory_builder_) << "Can't add LocalSlamResultData with "
                                         "local_trajectory_builder_ present.";
    local_slam_result_data->AddToPoseGraph(trajectory_id_, pose_graph_);
  }

 private:
  const int trajectory_id_;
  PoseGraph* const pose_graph_;
  std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder_;
  LocalSlamResultCallback local_slam_result_callback_;
  absl::optional<MotionFilter> pose_graph_odometry_motion_filter_;
};

}  // namespace


/**
 * @brief 新建一个2d全局轨迹生成器 
 * @param local_trajectory_builder     
 * @param pose_graph         
 * @param local_slam_result_callback
 * @param pose_graph_odometry_motion_filter 
 * */
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback,
    const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter) {
  return absl::make_unique<
      GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback, pose_graph_odometry_motion_filter);
}
/**
 * @brief 新建一个3d全局轨迹生成器 
 * @param local_trajectory_builder	   
 * @param pose_graph		 
 * @param local_slam_result_callback
 * @param pose_graph_odometry_motion_filter 
 * */
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D(
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph3D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback,
    const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter) {
  return absl::make_unique<
      GlobalTrajectoryBuilder<LocalTrajectoryBuilder3D, mapping::PoseGraph3D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback, pose_graph_odometry_motion_filter);
}

void GlobalTrajectoryBuilderRegisterMetrics(metrics::FamilyFactory* factory) {
  auto* results = factory->NewCounterFamily(
      "mapping_global_trajectory_builder_local_slam_results",
      "Local SLAM results");
  kLocalSlamMatchingResults = results->Add({{"type", "MatchingResult"}});
  kLocalSlamInsertionResults = results->Add({{"type", "InsertionResult"}});
}

}  // namespace mapping
}  // namespace cartographer
