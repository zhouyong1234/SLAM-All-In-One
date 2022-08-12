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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_

#include <bitset>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace sensor {
// 体素滤波器
// 滤波器
// points 待滤波的点
// resolution 分辨率
// 对RangeFinderPoint 进行滤波
std::vector<RangefinderPoint> VoxelFilter(
    const std::vector<RangefinderPoint>& points, const float resolution);
// 对PointCloud进行滤波
PointCloud VoxelFilter(const PointCloud& point_cloud, const float resolution);
// 对TimePointCloud 进行滤波
// 仅有采集数据时的整体信息
TimedPointCloud VoxelFilter(const TimedPointCloud& timed_point_cloud,
                            const float resolution);
// 对含有时间信息的点云进行滤波
// 每一个激光点都含有时间信息
std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> VoxelFilter(
    const std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement>&
        range_measurements,
    const float resolution);

// 自适应滤波器配置函数
proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary);
// 自适应滤波器
PointCloud AdaptiveVoxelFilter(
    const PointCloud& point_cloud,
    const proto::AdaptiveVoxelFilterOptions& options);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
