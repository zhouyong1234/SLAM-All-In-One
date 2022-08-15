#ifndef HDL_GLOBAL_LOCALIZATION_FPFH_RANSAC_HPP
#define HDL_GLOBAL_LOCALIZATION_FPFH_RANSAC_HPP

#include <ros/ros.h>

#include <hdl_global_localization/engines/global_localization_engine.hpp>
#include <hdl_global_localization/ransac/ransac_pose_estimation.hpp>

namespace hdl_global_localization {

class GlobalLocalizationEngineFPFH_RANSAC : public GlobalLocalizationEngine {
public:
  GlobalLocalizationEngineFPFH_RANSAC(ros::NodeHandle& private_nh);
  virtual ~GlobalLocalizationEngineFPFH_RANSAC() override;

  virtual void set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) override;
  virtual GlobalLocalizationResults query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) override;

protected:
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr extract_fpfh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

protected:
  ros::NodeHandle& private_nh;

  std::unique_ptr<RansacPoseEstimation<pcl::FPFHSignature33>> ransac;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr global_map;
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr global_map_features;
};

}  // namespace hdl_global_localization

#endif