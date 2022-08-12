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

#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
constexpr int kSubpixelScale = 1000;

// 在插入激光数据之前调用
void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid) {
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  // Padding around bounding box to avoid numerical issues at cell boundaries.
  constexpr float kPadding = 1e-6f;
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    bounding_box.extend(hit.position.head<2>());
  }
  for (const sensor::RangefinderPoint& miss : range_data.misses) {
    bounding_box.extend(miss.position.head<2>());
  }
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}

// 往概率栅格地图中插入激光数据核心调用函数
// range_data          激光数据
// hit_table           击中cost表
// miss_table          miss cost表
// insert_free_space   插入空闲空间
// probability_grid    概率栅格地图指针
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space, ProbabilityGrid* probability_grid) {
              
  GrowAsNeeded(range_data, probability_grid);

  const MapLimits& limits = probability_grid->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // 添加端点
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;// 端点向量
  ends.reserve(range_data.returns.size());// 大小
  // 遍历所有端点
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
  	// 计算对应的id 
    ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
	// 概率栅格地图更新
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }

  if (!insert_free_space) {
    return;
  }

  // Now add the misses.
  // 添加超过的点
  for (const Eigen::Array2i& end : ends) {
  	// 计算超过范围的点的激光路径
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale);
	// 更新该路径上的概率
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }

  // Finally, compute and add empty rays based on misses in the range data.
  // 根据端点计算激光路径
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
  	// 更新击中点的激光路径
    std::vector<Eigen::Array2i> ray = RayToPixelMask(
        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
	// 更新该路径上的概率
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }
}
}  // namespace

proto::ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space")
          ? parameter_dictionary->GetBool("insert_free_space")
          : true);
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

// 构造函数
// 调用函数 ComputeLookupTableToApplyCorrespondenceCostOdds 得到hit_table_  变量
// 调用函数 ComputeLookupTableToApplyCorrespondenceCostOdds 得到miss_table_ 变量
ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const proto::ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.hit_probability()))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.miss_probability()))) {}

// 核心实现函数，插入激光数据到栅格地图中
// range_data  激光数据
// grid        栅格地图
// 调用函数     CastRays 实现功能 
void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data, GridInterface* const grid) const {
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  // 调用函数 CastRays 
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(),
           probability_grid);
  probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
