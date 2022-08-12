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

#ifndef CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
#define CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace sensor {

// 含时间的点云数据
// 采集点云的时间
struct TimedPointCloudData {
  common::Time time;// 时间
  Eigen::Vector3f origin;// 原点
  TimedPointCloud ranges;// 3d激光点
  // 'intensities' has to be same size as 'ranges', or empty.
  std::vector<float> intensities;// 强度
};

// 含时间原点的点云数据
// 每一个3D激光点都有对应的时间
struct TimedPointCloudOriginData {
  struct RangeMeasurement {
    TimedRangefinderPoint point_time;// 位置+时间
    float intensity; // 强度
    size_t origin_index;// id
  };
  common::Time time;// 时间
  std::vector<Eigen::Vector3f> origins;// 原点
  std::vector<RangeMeasurement> ranges;// 点云
};

// Converts 'timed_point_cloud_data' to a proto::TimedPointCloudData.
proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data);

// Converts 'proto' to TimedPointCloudData.
TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
