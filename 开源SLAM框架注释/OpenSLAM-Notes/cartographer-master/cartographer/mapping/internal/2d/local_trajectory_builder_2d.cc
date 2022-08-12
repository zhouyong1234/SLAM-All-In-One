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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

// 构造函数，单纯的赋值
LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids) {}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}


// 将经过运动畸变矫正后的激光数据对齐并进行滤波处理
// transform_to_gravity_aligned_frame 重力对齐的方向
// range_data 激光雷达的数据
// 返回经过处理后的激光雷达数据
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const
{
  // 对数据进行旋转并限幅
  const sensor::RangeData cropped =
      sensor::CropRangeData(sensor::TransformRangeData(
                                range_data, transform_to_gravity_aligned_frame),
                            options_.min_z(), options_.max_z());
  // 分别对misses 和 returns 进行滤波
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFilter(cropped.returns, options_.voxel_filter_size()),
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};
}

// CSM算法
// time 激光数据采集时间
// pose_prediction 位置初始值
// filtered_gravity_aligned_point_cloud 激光数据
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud)
{
  // 没有激活状态的子地图
  if (active_submaps_.submaps().empty())
  { 
    // 预测位置即为当前匹配的位置
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // 获取得到最新的子地图指针
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  // 初始值赋值
  transform::Rigid2d initial_ceres_pose = pose_prediction;

  if (options_.use_online_correlative_scan_matching())
  {// 使用在线的匹配算法
    // 调用在线匹配算法，得到一个初始解，这样一个解是全局最优的
    // 搜索范围需要调整，有imu角度范围就可以给小一点，
    //没有则需要给较大的搜索范围才能保证是全局最优的 
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *matching_submap->grid(), &initial_ceres_pose);
	// 
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  // 调用ceres_scan_matcher_ 进一步得到更加精切的值
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary);
  if (pose_observation)
  {
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    const double residual_distance =
        (pose_observation->translation() - pose_prediction.translation())
            .norm();
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle =
        std::abs(pose_observation->rotation().angle() -
                 pose_prediction.rotation().angle());
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }
  return pose_observation;
}


// 添加激光雷达数据
// sensor_id 传感器id 
// unsynchronized_data 未同步的数据
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) 
{
  // 构造一个同步的数据结构
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }
  // 当前时间
  const common::Time& time = synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
  if (!options_.use_imu_data()) 
  { // 初始化估计器，该初始化操作仅会被执行一次
    InitializeExtrapolator(time);
  }
  // 确保初始化完成
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }
  
  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);
  // 采集第一个点的数据的时间
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);
  // 
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }
  // 获得激光数据时的估计位置
  std::vector<transform::Rigid3f> range_data_poses;
  // 分配空间
  range_data_poses.reserve(synchronized_data.ranges.size());
  bool warned = false;
  // 遍历所有激光数据
  for (const auto& range : synchronized_data.ranges) 
  {
    // 当前激光数据的时间 
    common::Time time_point = time + common::FromSeconds(range.point_time.time);
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) 
	{
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
	// 使用运动估计器得到采集该激光点数据时的估计位置
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  // 遍历所有的激光点数据
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) 
  {
    // 时间 
    const sensor::TimedRangefinderPoint& hit =
        synchronized_data.ranges[i].point_time;
	// 发射激光的位置
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] *
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);
    // 击中的位置 
	sensor::RangefinderPoint hit_in_local =
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    const float range = delta.norm();
    if (range >= options_.min_range()) 
	{
      if (range <= options_.max_range()) 
	  {// 在最大值、最小值范围内的数据认为时return
        accumulated_range_data_.returns.push_back(hit_in_local);
      } 
	  else 
	  {
	    // 超过最大值 
        hit_in_local.position =
            origin_in_local +
            options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }
  ++num_accumulated_;
  // 点的数量足够
  if (num_accumulated_ >= options_.num_accumulated_range_data())
  {
    const common::Time current_sensor_time = synchronized_data.time;
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value())
	{
      sensor_duration = current_sensor_time - last_sensor_time_.value();
    }
    last_sensor_time_ = current_sensor_time;
    num_accumulated_ = 0;
	// IMU检测重力得到的旋转
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));
    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    accumulated_range_data_.origin = range_data_poses.back().translation();
    // 调用函数 AddAccumulatedRangeData
    // time 当前时间
    // TransformToGravityAlignedFrameAndFilter 转换到重力方向并滤波的激光数据
    // gravity_alignment 重力对齐得到的旋转方向
    // sensor_duration 传感器数据持续时间
	return AddAccumulatedRangeData(
        time,
        TransformToGravityAlignedFrameAndFilter(
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
            accumulated_range_data_),
        gravity_alignment, sensor_duration);
  }
  return nullptr;
}


// 添加经过处理后的激光雷达数据
// time 采集时间
// gravity_aligned_range_data 重力方向对齐后并滤波的激光数据
// gravity_alignment 重力方向
// sensor_duration 传感器持续时间
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration) 
{
  // 如果激光没有放置水平，拒绝调用本函数 
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  // 没有imu数据则使用估计器的值作为初始值
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time);
  // 有imu数据则采用重力方向估计的位姿作为初始值
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());
  // 自适应滤波
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns,
                                  options_.adaptive_voxel_filter_options());

  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  // local map frame <- gravity-aligned frame
  // 使用CSM跟踪位姿，使用pose_prediction作为初始值
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);
  if (pose_estimate_2d == nullptr) 
  {// CSM算法失败了
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  // 使用当前结果更新状态估计器的位姿信息，2d-->3d
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  extrapolator_->AddPose(time, pose_estimate);
  // 根据匹配结果更新激光点的位置
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  // 插入到子地图中
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_estimate, gravity_alignment.rotation());
  // 获取当前时间
  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value())// 不是第一帧时执行
  {
    // 与上一帧数据时间差
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    // kLocalSlamLatencyMetric
	kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value())
	{
	  // kLocalSlamRealTimeRatio 
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  // 记录时间
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
  // 返回匹配状态
  return absl::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

// 将CSM匹配得到的激光数据插入到子地图中
// time 时间
// range_data_in_local 使用匹配结果更新激光点后的位置
// filtered_gravity_aligned_point_cloud 输入的点云
// pose_estimate 平移估计
// gravity_alignment 重力对齐方向 
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) 
{
  // 时间太短，距离太小就不更新
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // 插入到子地图中
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),
      std::move(insertion_submaps)});
}

// 添加IMU数据 
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data)
{
  // 确保当前配置中使用imu
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  // 初始化 
  InitializeExtrapolator(imu_data.time);
  // 添加IMU数据到运动估计器中
  extrapolator_->AddImuData(imu_data);
}

// 添加里程计数据
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data)
{
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  // 添加到运动估计器中
  extrapolator_->AddOdometryData(odometry_data);
}

// 初始化估计器
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) 
{
  // 非空说明已经初始化过了，直接返回
  if (extrapolator_ != nullptr) {
    return;
  }
  // 只有使用imu的情况下需要
  CHECK(!options_.pose_extrapolator_options().use_imu_based());
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  // new 一个 位置估计器
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(options_.pose_extrapolator_options()
                                              .constant_velocity()
                                              .pose_queue_duration()),
      options_.pose_extrapolator_options()
          .constant_velocity()
          .imu_gravity_time_constant());
  // 当前位置为零点
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
