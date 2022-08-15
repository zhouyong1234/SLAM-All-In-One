#include <hdl_global_localization/ransac/voxelset.hpp>

#include <unordered_set>
#include <boost/functional/hash.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hdl_global_localization {

size_t Vector3iHash::operator()(const Eigen::Vector3i& x) const {
  size_t seed = 0;
  boost::hash_combine(seed, x[0]);
  boost::hash_combine(seed, x[1]);
  boost::hash_combine(seed, x[2]);
  return seed;
}

VoxelSet::VoxelSet(double max_correspondence_distance) : resolution(max_correspondence_distance) {}

void VoxelSet::set_cloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  voxels.clear();

  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> offsets;
  offsets.reserve(9);
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      for (int k = -1; k <= 1; k++) {
        offsets.push_back(Eigen::Vector3i(i, j, k));
      }
    }
  }

  for (const auto& pt : *cloud) {
    Eigen::Vector3i coord = voxel_coord(pt.getVector4fMap());
    for (const auto& offset : offsets) {
      Eigen::Vector4f center = voxel_center(coord + offset);
      if ((center - pt.getVector4fMap()).squaredNorm() < resolution * resolution) {
        voxels.insert(coord + offset);
      }
    }
  }
}

double VoxelSet::matching_error(const pcl::PointCloud<pcl::PointXYZ>& cloud, double* inlier_fraction) const {
  int num_inliers = 0;
  double errors = 0.0;

  for (const auto& pt : cloud) {
    Eigen::Vector3i coord = voxel_coord(pt.getVector4fMap());
    if (voxels.find(coord) != voxels.end()) {
      Eigen::Vector4f center = voxel_center(coord);
      num_inliers++;
      errors += (pt.getVector4fMap() - center).squaredNorm();
    }
  }

  *inlier_fraction = static_cast<double>(num_inliers) / cloud.size();
  return num_inliers ? errors / num_inliers : std::numeric_limits<double>::max();
}

Eigen::Vector3i VoxelSet::voxel_coord(const Eigen::Vector4f& x) const {
  return (x.array() / resolution - 0.5).floor().cast<int>().head<3>();
}

Eigen::Vector4f VoxelSet::voxel_center(const Eigen::Vector3i& coord) const {
  Eigen::Vector3f origin = (coord.template cast<float>().array() + 0.5) * resolution;
  Eigen::Vector3f offset = Eigen::Vector3f::Ones() * resolution / 2.0f;
  return (Eigen::Vector4f() << origin + offset, 1.0).finished();
}

}  // namespace hdl_global_localization
