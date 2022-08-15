#ifndef HDL_GLOBAL_LOCALIZATION_MATCHING_COST_EVALUATER_FLANN_HPP
#define HDL_GLOBAL_LOCALIZATION_MATCHING_COST_EVALUATER_FLANN_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <hdl_global_localization/ransac/matching_cost_evaluater.hpp>

namespace hdl_global_localization {

class MatchingCostEvaluaterFlann : public MatchingCostEvaluater {
public:
  MatchingCostEvaluaterFlann() {}
  virtual ~MatchingCostEvaluaterFlann() override {}

  virtual void set_target(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double max_correspondence_distance) override {
    max_correspondence_distance_sq = max_correspondence_distance * max_correspondence_distance;

    tree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
  }

  virtual double calc_matching_error(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Matrix4f& transformation, double* inlier_fraction) override {
    int num_inliers = 0;
    double matching_error = 0.0;

    pcl::PointCloud<pcl::PointXYZ> transformed;
    pcl::transformPointCloud(cloud, transformed, transformation);

    std::vector<int> indices;
    std::vector<float> sq_dists;
    for (int i = 0; i < transformed.size(); i++) {
      tree->nearestKSearch(transformed[i], 1, indices, sq_dists);
      if (sq_dists[0] < max_correspondence_distance_sq) {
        num_inliers++;
        matching_error += sq_dists[0];
      }
    }

    if (inlier_fraction) {
      *inlier_fraction = static_cast<double>(num_inliers) / cloud.size();
    }
    return num_inliers ? matching_error / num_inliers : std::numeric_limits<double>::max();
  }

private:
  double max_correspondence_distance_sq;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree;
};

}  // namespace hdl_global_localization

#endif