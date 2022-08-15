#include <hdl_global_localization/engines/global_localization_bbs.hpp>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <hdl_global_localization/bbs/bbs_localization.hpp>
#include <hdl_global_localization/bbs/occupancy_gridmap.hpp>

namespace hdl_global_localization {

GlobalLocalizationBBS::GlobalLocalizationBBS(ros::NodeHandle& private_nh) : private_nh(private_nh) {
  gridmap_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("bbs/gridmap", 1, true);
  map_slice_pub = private_nh.advertise<sensor_msgs::PointCloud2>("bbs/map_slice", 1, true);
  scan_slice_pub = private_nh.advertise<sensor_msgs::PointCloud2>("bbs/scan_slice", 1, false);
}

GlobalLocalizationBBS ::~GlobalLocalizationBBS() {}

void GlobalLocalizationBBS::set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  BBSParams params;
  params.max_range = private_nh.param<double>("bbs/max_range", 15.0);
  params.min_tx = private_nh.param<double>("bbs/min_tx", -10.0);
  params.max_tx = private_nh.param<double>("bbs/max_tx", 10.0);
  params.min_ty = private_nh.param<double>("bbs/min_ty", -10.0);
  params.max_ty = private_nh.param<double>("bbs/max_ty", 10.0);
  params.min_theta = private_nh.param<double>("bbs/min_theta", -3.15);
  params.max_theta = private_nh.param<double>("bbs/max_theta", 3.15);
  bbs.reset(new BBSLocalization(params));

  double map_min_z = private_nh.param<double>("bbs/map_min_z", 2.0);
  double map_max_z = private_nh.param<double>("bbs/map_max_z", 2.4);
  auto map_2d = slice(*cloud, map_min_z, map_max_z);
  ROS_INFO_STREAM("Set Map " << map_2d.size() << " points");

  if (map_2d.size() < 128) {
    ROS_WARN_STREAM("Num points in the sliced map is too small!!");
    ROS_WARN_STREAM("Change the slice range parameters!!");
  }

  int map_width = private_nh.param<int>("bbs/map_width", 1024);
  int map_height = private_nh.param<int>("bbs/map_height", 1024);
  double map_resolution = private_nh.param<double>("bbs/map_resolution", 0.5);
  int map_pyramid_level = private_nh.param<int>("bbs/map_pyramid_level", 6);
  int max_points_per_cell = private_nh.param<int>("bbs/max_points_per_cell", 5);
  bbs->set_map(map_2d, map_resolution, map_width, map_height, map_pyramid_level, max_points_per_cell);

  auto map_3d = unslice(map_2d);
  map_3d->header.frame_id = "map";
  map_slice_pub.publish(map_3d);
  gridmap_pub.publish(bbs->gridmap()->to_rosmsg());
}

GlobalLocalizationResults GlobalLocalizationBBS::query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) {
  double scan_min_z = private_nh.param<double>("bbs/scan_min_z", -0.2);
  double scan_max_z = private_nh.param<double>("bbs/scan_max_z", 0.2);
  auto scan_2d = slice(*cloud, scan_min_z, scan_max_z);

  std::vector<GlobalLocalizationResult::Ptr> results;

  ROS_INFO_STREAM("Query " << scan_2d.size() << " points");
  if (scan_2d.size() < 32) {
    ROS_WARN_STREAM("Num points in the sliced scan is too small!!");
    ROS_WARN_STREAM("Change the slice range parameters!!");
    return GlobalLocalizationResults(results);
  }

  double best_score = 0.0;
  auto trans_2d = bbs->localize(scan_2d, 0.0, &best_score);
  if (trans_2d == boost::none) {
    return GlobalLocalizationResults(results);
  }

  if (scan_slice_pub.getNumSubscribers()) {
    auto scan_3d = unslice(scan_2d);
    scan_3d->header = cloud->header;
    scan_slice_pub.publish(scan_3d);
  }

  Eigen::Isometry3f trans_3d = Eigen::Isometry3f::Identity();
  trans_3d.linear().block<2, 2>(0, 0) = trans_2d->linear();
  trans_3d.translation().head<2>() = trans_2d->translation();

  results.resize(1);
  results[0].reset(new GlobalLocalizationResult(best_score, best_score, trans_3d));

  return GlobalLocalizationResults(results);
}

GlobalLocalizationBBS::Points2D GlobalLocalizationBBS::slice(const pcl::PointCloud<pcl::PointXYZ>& cloud, double min_z, double max_z) const {
  Points2D points_2d;
  points_2d.reserve(cloud.size());
  for (int i = 0; i < cloud.size(); i++) {
    if (min_z < cloud.at(i).z && cloud.at(i).z < max_z) {
      points_2d.push_back(cloud.at(i).getVector3fMap().head<2>());
    }
  }
  return points_2d;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GlobalLocalizationBBS::unslice(const Points2D& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->resize(points.size());
  for (int i = 0; i < points.size(); i++) {
    cloud->at(i).getVector3fMap().head<2>() = points[i];
    cloud->at(i).z = 0.0f;
  }

  return cloud;
}
}  // namespace hdl_global_localization