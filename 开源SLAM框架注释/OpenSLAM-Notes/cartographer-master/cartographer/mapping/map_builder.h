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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"

namespace cartographer {
namespace mapping {

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
// 连接所有局部地图和用于回环检测的位姿图
class MapBuilder : public MapBuilderInterface {
 public:
  // 构造函数	
  explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder() override {}
  // 复制构造函数 
  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;
  // 添加轨迹构建类
  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;
  // 添加反序列化的轨迹
  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;
  // 标记轨迹完成
  void FinishTrajectory(int trajectory_id) override;
  // 子地图转换到Proto格式
  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;
  // 序列化状态
  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriterInterface *writer) override;
  // 序列化状态写入到文件中
  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename) override;
  // 从proto格式中载入状态
  std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                               bool load_frozen_state) override;
  // 从文件中载入状态
  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       const bool load_frozen_state) override;
  // 获取位姿图
  mapping::PoseGraphInterface *pose_graph() override {
    return pose_graph_.get();
  }
  // 获取轨迹生成类的数量
  int num_trajectory_builders() const override {
    return trajectory_builders_.size();
  }
  // 获取trajectory_id对应的轨迹生成类
  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const override {
    return trajectory_builders_.at(trajectory_id).get();
  }
  // 获取轨迹生成类的配置选项
  const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const override {
    return all_trajectory_builder_options_;
  }

 private:
  // 地图构造选项
  const proto::MapBuilderOptions options_;
  // 线程池
  common::ThreadPool thread_pool_;
  // 位姿图指针
  std::unique_ptr<PoseGraph> pose_graph_;
  // 传感器
  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
  // 轨迹生成类指针向量
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_;
  // 轨迹生成配置选项向量
  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      all_trajectory_builder_options_;
};

// 实际创建类时调用的函数
std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
    const proto::MapBuilderOptions& options);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
