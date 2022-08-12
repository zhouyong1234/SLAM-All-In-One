/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// 构造函数，单纯的赋值
PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}

// 使用IMU数据初始化位姿估计器
// pose_queue_duration          
// imu_gravity_time_constant   IMU保持静止的时间
// imu_data                    IMU 数据
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) 
{
  // 新构造一个全新的位姿估计器  
  auto extrapolator = absl::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  // 添加IMU数据
  extrapolator->AddImuData(imu_data);
  // 新构造一个 ImuTracker
  extrapolator->imu_tracker_ =
      absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  // 添加加速度计数据
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  // 添加角加速度数据
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  // imu_tracker_得到旋转数据
  extrapolator->imu_tracker_->Advance(imu_data.time);
  // 初始位姿为使用imu得到的旋转
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

// 获取上一次位姿的时间
common::Time PoseExtrapolator::GetLastPoseTime() const 
{
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}
// 获取上一次获取imu数据的时间
common::Time PoseExtrapolator::GetLastExtrapolatedTime() const 
{
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}
// 添加初始位姿数据
// time 初始时刻
// pose 初始位姿
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) 
{
  // imu 跟踪器没有初始化
  if (imu_tracker_ == nullptr)
  {
    // 初始化imu跟踪器
    common::Time tracker_start = time;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    imu_tracker_ =
        absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  // 将位姿数据记录 
  timed_pose_queue_.push_back(TimedPose{time, pose});
  // 仅仅留下最新的位姿数据
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  // 调用函数更新状态估计器初始值 
  UpdateVelocitiesFromPoses();
  AdvanceImuTracker(time, imu_tracker_.get());
  TrimImuData();
  TrimOdometryData();
  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}
							   
// 添加IMU数据
// imu_data imu的数据
// 记录数据和时间后调用 TrimImuData
void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data)
{
  // 插入imu数据获取时间 
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  // 插入imu数据
  imu_data_.push_back(imu_data);
  // 调用TrimImuData
  TrimImuData();
}
// 添加里程计数据
// odometry_data 里程计数据
// 通过差值估计当前时间的角速度和线速度值
// 这里没有直接使用位置数据，估计是认为里程计数据有滑移，认为是速度信息更准确一点
void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) 
{
  // 记录里程计数据和时间，然后调用TrimOdometryData
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO 通过使用多个（而不是最后两个）测距姿势来进行改进在跟踪框架中计算外推。
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  // 上一次的里程计数据	
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  // 当前里程计数据
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  // 相差的时间	
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  // 相差的位姿
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  // 得到角速度
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  // 坐标变换
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
  // 线速度值
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

// 获取time时刻的估计位姿
transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time)
{
  // 最新的位姿时间
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);

  if (cached_extrapolated_pose_.time != time)
  {// 如果不是之前估计得到的时间
    // 平移
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
    // 旋转
	const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    // 最终得到time时刻的位姿
	cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}
// 估计重力方向
Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) 
{
  // 直接新建一个imu跟踪器
  ImuTracker imu_tracker = *imu_tracker_;
  // 计算方向
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}

// 从位姿中更新速度
void PoseExtrapolator::UpdateVelocitiesFromPoses() 
{
  // 位姿数少于两个无法计算速度
  if (timed_pose_queue_.size() < 2)
  {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  // 估计得到的线速度
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  // 估计得到的角速度	
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}

// 更新imu数据
void PoseExtrapolator::TrimImuData() 
{
  // 保留最新的一帧imu数据
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time)
  {
    imu_data_.pop_front();
  }
}
// 更新odometry数据
void PoseExtrapolator::TrimOdometryData() 
{
  // 保留最新的两帧里程计数据
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}


// 更新imu跟踪器
// time 时间
// imu_tracker imu跟踪器
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const
{
  // 确保当前时间较新
  CHECK_GE(time, imu_tracker->time());
  if (imu_data_.empty() || time < imu_data_.front().time) 
  {// 没有imu数据的情况下
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  if (imu_tracker->time() < imu_data_.front().time)
  {// 有imu数据的情况下
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  // 获取小于time的第一个值
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  // 
  while (it != imu_data_.end() && it->time < time) 
  {
    // 添加time时刻之前的所有值 
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}

// 估计time时刻的旋转
Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const
{
  CHECK_GE(time, imu_tracker->time());
  AdvanceImuTracker(time, imu_tracker);
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  // 利用imu数据估计旋转角度
  return last_orientation.inverse() * imu_tracker->orientation();
}

// 估计time时刻的平移
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) 
{
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  // 时间 * 线速度
  return extrapolation_delta * linear_velocity_from_odometry_;
}


// 使用重力方向来估计位姿
PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {
  std::vector<transform::Rigid3f> poses;
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
    poses.push_back(ExtrapolatePose(*it).cast<float>());
  }

  const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
                                               ? linear_velocity_from_poses_
                                               : linear_velocity_from_odometry_;
  return ExtrapolationResult{poses, ExtrapolatePose(times.back()),
                             current_velocity,
                             EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace cartographer
