#ifndef HDL_GLOBAL_LOCALIZATION_VOXELSET_HPP
#define HDL_GLOBAL_LOCALIZATION_VOXELSET_HPP

#include <unordered_set>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hdl_global_localization {

class Vector3iHash {
public:
  size_t operator()(const Eigen::Vector3i& x) const;
};

class VoxelSet {
public:
  VoxelSet(double max_correspondence_distance);

  void set_cloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
  double matching_error(const pcl::PointCloud<pcl::PointXYZ>& cloud, double* inlier_fraction) const;

private:
  Eigen::Vector3i voxel_coord(const Eigen::Vector4f& x) const;
  Eigen::Vector4f voxel_center(const Eigen::Vector3i& coord) const;

private:
  double resolution;

  using Voxels = std::unordered_set<Eigen::Vector3i, Vector3iHash, std::equal_to<Eigen::Vector3i>, Eigen::aligned_allocator<Eigen::Vector3i>>;
  Voxels voxels;
};  // namespace hdl_global_localization

}  // namespace hdl_global_localization

#endif