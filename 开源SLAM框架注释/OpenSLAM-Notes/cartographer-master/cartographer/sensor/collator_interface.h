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

#ifndef CARTOGRAPHER_SENSOR_COLLATOR_INTERFACE_H_
#define CARTOGRAPHER_SENSOR_COLLATOR_INTERFACE_H_

#include <functional>
#include <memory>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/types/optional.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

// 整理接口
// 根据配置参数确定使用 TrajectoryCollator 或者 Collator
class CollatorInterface {
 public:
  // 回调函数 
  // param1 - string  传感器id
  // param2 - Data    传感器数据
  using Callback =
      std::function<void(const std::string&, std::unique_ptr<Data>)>;
  // 默认构造，析构，复制函数
  CollatorInterface() {}
  virtual ~CollatorInterface() {}
  CollatorInterface(const CollatorInterface&) = delete;
  CollatorInterface& operator=(const CollatorInterface&) = delete;
  
  // Adds a trajectory to produce sorted sensor output for. Calls 'callback'
  // for each collated sensor data.
  // 添加一个轨迹，调用回调函数得到传感器数据
  // 轨迹id 
  // 传感器id
  // 回调函数
  virtual void AddTrajectory(
      int trajectory_id,
      const absl::flat_hash_set<std::string>& expected_sensor_ids,
      const Callback& callback) = 0;

  // Marks 'trajectory_id' as finished.
  // 标记 trajectory_id 为完成了
  virtual void FinishTrajectory(int trajectory_id) = 0;

  // Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
  // sensor data. Sensor packets with matching 'data.sensor_id_' must be added
  // in time order.
  // 添加传感器数据到trajectory_id对应的轨迹中
  virtual void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) = 0;

  // Dispatches all queued sensor packets. May only be called once.
  // 调度所有排队的传感器数据包，只能被调用一次
  // AddSensorData may not be called after Flush.
  // 调用Flush后可能不会再调用AddSensorData函数了
  virtual void Flush() = 0;

  // Must only be called if at least one unfinished trajectory exists. Returns
  // the ID of the trajectory that needs more data before CollatorInterface is
  // unblocked. Returns 'nullopt' for implementations that do not wait for a
  // particular trajectory.
  // 仅当存在至少一条未完成的轨迹时才能调用。返回需要更多数据的轨迹id。对于特定的
  // 不等待输入的实现，返回"nullptr"
  virtual absl::optional<int> GetBlockingTrajectoryId() const = 0;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COLLATOR_INTERFACE_H_
