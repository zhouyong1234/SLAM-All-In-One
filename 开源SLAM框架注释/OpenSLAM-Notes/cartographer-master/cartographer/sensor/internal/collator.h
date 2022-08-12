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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_

#include <functional>
#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/internal/ordered_multi_queue.h"

namespace cartographer {
namespace sensor {
// 传感器数据统一调度类
class Collator : public CollatorInterface {
 public:
  // 默认构造、析构、复制构造函数	
  Collator() {}
  Collator(const Collator&) = delete;
  Collator& operator=(const Collator&) = delete;
  // 添加一条轨迹
  void AddTrajectory(
      int trajectory_id,
      const absl::flat_hash_set<std::string>& expected_sensor_ids,
      const Callback& callback) override;
  // 标记trajectory_id结束
  void FinishTrajectory(int trajectory_id) override;
  // 添加传感器数据
  void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;
  // 刷新
  void Flush() override;
  // 回去未完成的轨迹id
  absl::optional<int> GetBlockingTrajectoryId() const override;

 private:
  // Queue keys are a pair of trajectory ID and sensor identifier.
  // 队列的关键码是一对轨迹id和传感器id
  OrderedMultiQueue queue_;

  // Map of trajectory ID to all associated QueueKeys.
  // 轨迹id到所有关键的队列关键码
  absl::flat_hash_map<int, std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_
