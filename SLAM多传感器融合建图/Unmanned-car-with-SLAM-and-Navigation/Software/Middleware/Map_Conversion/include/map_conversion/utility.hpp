//
// Created by qjs on 21-5-11.
//

#ifndef MAP_CONVERSION_UTILITY_H
#define MAP_CONVERSION_UTILITY_H

#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <string>
#include "string.h"
#include <iostream>
#include <Eigen/Eigen>
#include <mutex>
#include <deque>
#include <thread>
#include <yaml-cpp/yaml.h>
#include "glog/logging.h"
#include "map_conversion/global_defination/global_defination.h"
#include "map_conversion/ros_topic_interface/cloud_subscriber.hpp"
#include "map_conversion/pointcloud_process/pointcloud_2d_process.hpp"
#include "map_conversion/pointcloud_process/pointcloud_3d_process.hpp"
#include "map_conversion/ros_topic_interface/cloud_publisher.hpp"


using namespace map_conversion;
using namespace Eigen;
using namespace std;

struct GridNode
{     
   int id;        // 1--> open set, -1 --> closed set
   Eigen::Vector3d coord;
   Eigen::Vector3i index;
   
   uint8_t * occupancy; 

   GridNode(Eigen::Vector3i _index)
   {  
      id = 0;
      index = _index;
   }

   GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord)
   {  
      id = 0;
      index = _index;
      coord = _coord;
   }

   GridNode(){};
   
   ~GridNode(){};
};

#endif //OFFLINE_FUSION_UTILITY_H
