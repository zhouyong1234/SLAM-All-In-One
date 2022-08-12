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

#include "cartographer/sensor/point_cloud.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {
// 默认构造函数
PointCloud::PointCloud() {}
// 构造函数
// points 点云向量
PointCloud::PointCloud(std::vector<PointCloud::PointType> points)
    : points_(std::move(points)) {}
// 构造函数
// points 点云向量
// intensities 点云强度,如果点云强度非空则大小应该和点云一致
PointCloud::PointCloud(std::vector<PointType> points,
                       std::vector<float> intensities)
    : points_(std::move(points)), intensities_(std::move(intensities)) {
  if (!intensities_.empty()) {
  	// 确保二者大小一致
    CHECK_EQ(points_.size(), intensities_.size());
  }
}
// 点云的数量
size_t PointCloud::size() const { return points_.size(); }
// 点云是否为空
bool PointCloud::empty() const { return points_.empty(); }

// 获取点云中所有点
const std::vector<PointCloud::PointType>& PointCloud::points() const {
  return points_;
}
// 获取点云中所有点强度
const std::vector<float>& PointCloud::intensities() const {
  return intensities_;
}
// 重载运算符[],获取第index个激光点数据
const PointCloud::PointType& PointCloud::operator[](const size_t index) const {
  return points_[index];
}

// 第一个激光点数据
PointCloud::ConstIterator PointCloud::begin() const { return points_.begin(); }
// 最后一个激光点数据
PointCloud::ConstIterator PointCloud::end() const { return points_.end(); }

// 添加一个激光点数据
void PointCloud::push_back(PointCloud::PointType value) {
  points_.push_back(std::move(value));
}

// 对点云中所有点进行坐标变换
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform) {
  // 返回的点数据
  std::vector<RangefinderPoint> points;
  // 预先分配好空间，避免动态操作
  points.reserve(point_cloud.size());
  // 遍历所有点
  for (const RangefinderPoint& point : point_cloud.points()) {
  	// 记录变换后的点位置
    points.emplace_back(transform * point);
  }
  // 返回变换后的数据
  return PointCloud(points, point_cloud.intensities());
}

TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform) {
  TimedPointCloud result;
  result.reserve(point_cloud.size());
  for (const TimedRangefinderPoint& point : point_cloud) {
    result.push_back(transform * point);
  }
  return result;
}

PointCloud CropPointCloud(const PointCloud& point_cloud, const float min_z,
                          const float max_z) {
  return point_cloud.copy_if([min_z, max_z](const RangefinderPoint& point) {
    return min_z <= point.position.z() && point.position.z() <= max_z;
  });
}

}  // namespace sensor
}  // namespace cartographer
