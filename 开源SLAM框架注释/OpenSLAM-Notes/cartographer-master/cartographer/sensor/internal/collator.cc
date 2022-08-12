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

#include "cartographer/sensor/internal/collator.h"

namespace cartographer {
namespace sensor {
/**
 * @brief 添加一条轨迹
 * @param trajectory_id 轨迹id
 * @param expected_sensor_ids 传感器id
 * @param callback 回调处理函数
 * */
void Collator::AddTrajectory(
    const int trajectory_id,
    const absl::flat_hash_set<std::string>& expected_sensor_ids,
    const Callback& callback)
{
  for (const auto& sensor_id : expected_sensor_ids)
  {// 遍历所有传感器id
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
	// 添加到队列中
    queue_.AddQueue(queue_key,
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });
	// 				
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}

void Collator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    queue_.MarkQueueAsFinished(queue_key);
  }
}

void Collator::AddSensorData(const int trajectory_id,
                             std::unique_ptr<Data> data) {
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  queue_.Add(std::move(queue_key), std::move(data));
}

void Collator::Flush() { queue_.Flush(); }

absl::optional<int> Collator::GetBlockingTrajectoryId() const {
  return absl::optional<int>(queue_.GetBlocker().trajectory_id);
}

}  // namespace sensor
}  // namespace cartographer
