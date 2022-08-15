#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <hdl_global_localization/SetGlobalLocalizationEngine.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>

class GlobalLocalizationTestNode {
public:
  GlobalLocalizationTestNode() : nh() {
    set_engine_service = nh.serviceClient<hdl_global_localization::SetGlobalLocalizationEngine>("/hdl_global_localization/set_engine");
    set_global_map_service = nh.serviceClient<hdl_global_localization::SetGlobalMap>("/hdl_global_localization/set_global_map");
    query_service = nh.serviceClient<hdl_global_localization::QueryGlobalLocalization>("/hdl_global_localization/query");

    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 1, true);
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 1);
    points_sub = nh.subscribe("/velodyne_points", 1, &GlobalLocalizationTestNode::points_callback, this);
  }

  void set_engine(const std::string& engine_name) {
    hdl_global_localization::SetGlobalLocalizationEngine srv;
    srv.request.engine_name.data = engine_name;

    if (!set_engine_service.call(srv)) {
      ROS_INFO_STREAM("Failed to set global localization engine");
    }
  }

  void set_global_map(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    hdl_global_localization::SetGlobalMap srv;
    pcl::toROSMsg(*cloud, srv.request.global_map);

    if (!set_global_map_service.call(srv)) {
      ROS_INFO_STREAM("Failed to set global map");
    }

    cloud->header.frame_id = "map";
    globalmap_pub.publish(cloud);
  }

  void points_callback(sensor_msgs::PointCloud2ConstPtr cloud_msg) {
    ROS_INFO_STREAM("Callback");

    hdl_global_localization::QueryGlobalLocalization srv;
    srv.request.cloud = *cloud_msg;
    srv.request.max_num_candidates = 1;

    if (!query_service.call(srv) || srv.response.poses.empty()) {
      ROS_INFO_STREAM("Failed to find a global localization solution");
      return;
    }

    const auto& estimated = srv.response.poses[0];
    Eigen::Quaternionf quat(estimated.orientation.w, estimated.orientation.x, estimated.orientation.y, estimated.orientation.z);
    Eigen::Vector3f trans(estimated.position.x, estimated.position.y, estimated.position.z);

    Eigen::Isometry3f transformation = Eigen::Isometry3f::Identity();
    transformation.linear() = quat.toRotationMatrix();
    transformation.translation() = trans;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed, transformation);
    transformed->header.frame_id = "map";

    points_pub.publish(transformed);
  }

private:
  ros::NodeHandle nh;
  ros::ServiceClient set_engine_service;
  ros::ServiceClient set_global_map_service;
  ros::ServiceClient query_service;

  ros::Publisher globalmap_pub;

  ros::Publisher points_pub;
  ros::Subscriber points_sub;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "hdl_global_localization_test");

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(argv[1], *global_map);

  GlobalLocalizationTestNode node;
  // node.set_engine("FPFH_RANSAC");
  node.set_global_map(global_map);

  ros::spin();

  return 0;
}