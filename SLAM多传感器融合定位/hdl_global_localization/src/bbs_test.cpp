#include <random>
#include <iostream>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <glk/texture.hpp>
#include <glk/gridmap.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/pointcloud_buffer_pcl.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <hdl_global_localization/bbs/bbs_localization.hpp>
#include <hdl_global_localization/bbs/occupancy_gridmap.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr slice(const pcl::PointCloud<pcl::PointXYZ>& cloud, double min_height, double max_height) {
  auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filtered->header = cloud.header;
  for (const auto& pt : cloud) {
    if (min_height < pt.z && pt.z < max_height) {
      filtered->push_back(pt);
    }
  }
  return filtered;
}

std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> project_to_2d(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> points;
  for (const auto& pt : cloud) {
    points.push_back(pt.getVector3fMap().head<2>());
  }
  return points;
}

std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
transform_2d(const std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>& points, float tx, float ty, float theta) {
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> transformed(points.size());

  Eigen::Isometry2f trans = Eigen::Isometry2f::Identity();
  trans.linear() = Eigen::Rotation2Df(theta).toRotationMatrix();
  trans.translation() = Eigen::Vector2f(tx, ty);

  for (int i = 0; i < points.size(); i++) {
    transformed[i] = trans * points[i];
  }

  return transformed;
}

int main(int argc, char** argv) {
  auto map_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::io::loadPCDFile("/home/koide/catkin_ws/src/hdl_localization/data/map.pcd", *map_cloud);
  map_cloud->header.frame_id = "map";

  auto scan_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::io::loadPCDFile("/home/koide/Downloads/hdl_400/1509348797.060304000.pcd", *scan_cloud);
  scan_cloud->header.frame_id = "map";

  auto map_slice = slice(*map_cloud, 2.0, 2.4);
  auto map_2d = project_to_2d(*map_slice);

  auto scan_slice = slice(*scan_cloud, -0.2, 0.2);
  auto scan_2d = project_to_2d(*scan_slice);

  scan_2d = transform_2d(scan_2d, -5.5, 3.5, -2.0);

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> scan_3d(scan_2d.size());
  for (int i = 0; i < scan_2d.size(); i++) {
    scan_3d[i].head<2>() = scan_2d[i];
    scan_3d[i].z() = 0.0f;
  }

  hdl_global_localization::BBSLocalization bbs;
  bbs.set_map(map_2d, 0.5, 512, 1024, 5, 5);

  auto viewer = guik::LightViewer::instance();
  auto gridmap = bbs.gridmap();
  viewer->update_drawable(
    "map",
    std::make_shared<glk::GridMap>(gridmap->grid_resolution(), gridmap->width(), gridmap->height(), 1.0f, gridmap->data(), 1.0f, glk::GridMap::ColorMode::RAW),
    guik::TextureColor());
  viewer->update_drawable(
    "scan",
    std::make_shared<glk::PointCloudBuffer>(scan_3d[0].data(), sizeof(Eigen::Vector3f), scan_3d.size()),
    guik::FlatColor(Eigen::Vector4f::UnitY()).add("point_scale", 3.0f));
  viewer->spin_until_click();

  double best_score = 0.0;
  auto transformation = bbs.localize(scan_2d, 0.0, &best_score);

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> aligned(scan_2d.size());
  for (int i = 0; i < scan_2d.size(); i++) {
    aligned[i].head<2>() = (*transformation) * scan_2d[i];
    aligned[i].z() = 0.0f;
  }
  viewer->update_drawable(
    "scan",
    std::make_shared<glk::PointCloudBuffer>(aligned[0].data(), sizeof(Eigen::Vector3f), aligned.size()),
    guik::FlatColor(Eigen::Vector4f::UnitY()).add("point_scale", 3.0f));
  viewer->spin_until_click();

  return 0;
}