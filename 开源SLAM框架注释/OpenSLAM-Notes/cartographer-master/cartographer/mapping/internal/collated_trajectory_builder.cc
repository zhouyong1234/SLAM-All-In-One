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

#include "cartographer/mapping/internal/collated_trajectory_builder.h"

#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

/**
 * @brief CollatedTrajectoryBuilder  构造函数
 * @param trajectory_options		 轨迹生成选项
 * @param sensor_collator            传感器数据管理类
 * @param trajectory_id				 轨迹id 
 * @param expected_sensor_ids        传感器id 
 * @param wrapped_trajectory_builder 全局轨迹规划类
 * */
CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    const proto::TrajectoryBuilderOptions& trajectory_options,
    sensor::CollatorInterface* const sensor_collator, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      collate_landmarks_(trajectory_options.collate_landmarks()),
      collate_fixed_frame_(trajectory_options.collate_fixed_frame()),
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now())
{
  absl::flat_hash_set<std::string> expected_sensor_id_strings;
  for (const auto& sensor_id : expected_sensor_ids)
  {
    // 去掉landmark 数据
    if (sensor_id.type == SensorId::SensorType::LANDMARK &&
        !collate_landmarks_) 
    {
      continue;
    }
    // 去掉 fixed_frame_数据
    if (sensor_id.type == SensorId::SensorType::FIXED_FRAME_POSE &&
        !collate_fixed_frame_) 
    {
      continue;
    }
	// 将其他类型的数据传递到expected_sensor_id_strings中	
    expected_sensor_id_strings.insert(sensor_id.id);
  }
  // 添加轨迹
  // trajectory_id              轨迹id 
  // expected_sensor_id_strings 传感器类型 
  // HandleCollatedSensorData   回调处理函数
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_id_strings,
      [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });
}

// 添加传感器数据通用结构
void CollatedTrajectoryBuilder::AddData(std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}

// 处理传感器数据的回调函数
void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data)
{
  // 确保时间正确
  auto it = rate_timers_.find(sensor_id);
  if (it == rate_timers_.end()) {
    it = rate_timers_
             .emplace(
                 std::piecewise_construct, std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
  }
  it->second.Pulse(data->GetTime());

  if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }
  // 添加到轨迹生成类中
  // 调用全局轨迹规划类
  data->AddToTrajectoryBuilder(wrapped_trajectory_builder_.get());
}

}  // namespace mapping
}  // namespace cartographer
