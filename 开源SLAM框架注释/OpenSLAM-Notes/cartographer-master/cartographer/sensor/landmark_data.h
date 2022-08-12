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

#ifndef CARTOGRAPHER_SENSOR_LANDMARK_DATA_H_
#define CARTOGRAPHER_SENSOR_LANDMARK_DATA_H_

#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {
// 地标点观测数据
// 可以是人工贴的二维码
struct LandmarkObservation {
  std::string id;
  // 坐标变换关系
  transform::Rigid3d landmark_to_tracking_transform;
  double translation_weight;// 权重
  double rotation_weight;
};

// 地标点结构体
struct LandmarkData {
  common::Time time;// 观测时间
  // 同一时刻所有的观测到的地标点
  std::vector<LandmarkObservation> landmark_observations;// 观测点向量
};


// 数据格式转换
// Converts 'landmark_data' to a proto::LandmarkData.
proto::LandmarkData ToProto(const LandmarkData& landmark_data);

// Converts 'proto' to an LandmarkData.
LandmarkData FromProto(const proto::LandmarkData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_LANDMARK_DATA_H_
