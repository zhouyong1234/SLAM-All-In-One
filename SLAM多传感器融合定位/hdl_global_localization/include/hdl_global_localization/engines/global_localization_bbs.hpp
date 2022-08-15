#ifndef HDL_GLOBAL_LOCALIZATION_BBS_HPP
#define HDL_GLOBAL_LOCALIZATION_BBS_HPP

#include <ros/ros.h>

#include <hdl_global_localization/engines/global_localization_engine.hpp>

namespace hdl_global_localization {

class BBSLocalization;

class GlobalLocalizationBBS : public GlobalLocalizationEngine {
public:
  GlobalLocalizationBBS(ros::NodeHandle& private_nh);
  virtual ~GlobalLocalizationBBS() override;

  virtual void set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) override;

  virtual GlobalLocalizationResults query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) override;

private:
  using Points2D = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;
  Points2D slice(const pcl::PointCloud<pcl::PointXYZ>& cloud, double min_z, double max_z) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr unslice(const Points2D& points);

protected:
  ros::NodeHandle& private_nh;

  ros::Publisher gridmap_pub;
  ros::Publisher map_slice_pub;
  ros::Publisher scan_slice_pub;

  std::unique_ptr<BBSLocalization> bbs;
};

}  // namespace hdl_global_localization

#endif