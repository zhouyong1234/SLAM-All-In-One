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

#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"

#include "cartographer/mapping/probability_values.h"
#include "ceres/cubic_interpolation.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// Computes a cost for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
class OccupiedSpaceCostFunction2D 
{
 public:
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const sensor::PointCloud& point_cloud,
                              const Grid2D& grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        grid_(grid) {}
  // 误差计算函数，使用自动求导
  template <typename T>
  bool operator()(const T* const pose, T* residual) const 
  {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);// 平移
    Eigen::Rotation2D<T> rotation(pose[2]);// 旋转
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();// 旋转矩阵
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);// 最终的变换矩阵
    // 栅格地图的插值
    const GridArrayAdapter adapter(grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits& limits = grid_.limits();

    for (size_t i = 0; i < point_cloud_.size(); ++i)
	{
	  // 激光点位置
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].position.x())),
                                         (T(point_cloud_[i].position.y())),
                                         T(1.));
	  // 世界系下激光点位置
      const Eigen::Matrix<T, 3, 1> world = transform * point;
	  // 计算单个点的残差
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          &residual[i]);
	  // 残差项 
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  // 栅格插值
  class GridArrayAdapter 
  {
   public:
    enum { DATA_DIMENSION = 1 };
    // 默认构造函数
    explicit GridArrayAdapter(const Grid2D& grid) : grid_(grid) {}
    // 获取row和column对应的值
    void GetValue(const int row, const int column, double* const value) const 
    {
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {// 在地图范围外，则赋予最大的响应值
        *value = kMaxCorrespondenceCost;
      } else {
	  	// 获取栅格地图对应的代价
        *value = static_cast<double>(grid_.GetCorrespondenceCost(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }
    
    int NumRows() const {
      return grid_.limits().cell_limits().num_y_cells + 2 * kPadding;
    }
    
    int NumCols() const {
      return grid_.limits().cell_limits().num_x_cells + 2 * kPadding;
    }

   private:
    const Grid2D& grid_;// 栅格地图
  };

  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const Grid2D& grid_;
};

}  // namespace

ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const sensor::PointCloud& point_cloud,
    const Grid2D& grid) {
  return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                         ceres::DYNAMIC /* residuals */,
                                         3 /* pose variables */>(
      new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud, grid),
      point_cloud.size());
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
