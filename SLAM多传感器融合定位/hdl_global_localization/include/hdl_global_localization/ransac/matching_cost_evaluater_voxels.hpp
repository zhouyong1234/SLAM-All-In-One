#ifndef HDL_GLOBAL_LOCALIZATION_MATCHING_COST_EVALUATER_VOXELS_HPP
#define HDL_GLOBAL_LOCALIZATION_MATCHING_COST_EVALUATER_VOXELS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

#include <hdl_global_localization/ransac/voxelset.hpp>
#include <hdl_global_localization/ransac/matching_cost_evaluater.hpp>

namespace hdl_global_localization {

class MatchingCostEvaluaterVoxels : public MatchingCostEvaluater {
public:
  MatchingCostEvaluaterVoxels() {}
  virtual ~MatchingCostEvaluaterVoxels() override {}

  virtual void set_target(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double max_correspondence_distance) override {
    max_correspondence_distance_sq = max_correspondence_distance * max_correspondence_distance;

    voxels.reset(new VoxelSet(max_correspondence_distance));
    voxels->set_cloud(cloud);
  }

  virtual double calc_matching_error(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Matrix4f& transformation, double* inlier_fraction) override {
    int num_inliers = 0;
    double matching_error = 0.0;

    pcl::PointCloud<pcl::PointXYZ> transformed;
    pcl::transformPointCloud(cloud, transformed, transformation);

    return voxels->matching_error(transformed, inlier_fraction);
  }

private:
  double max_correspondence_distance_sq;
  std::unique_ptr<VoxelSet> voxels;
};

}  // namespace hdl_global_localization

#endif