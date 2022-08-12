#include <cmath>
#include <vector>
#include <string>
#include <time.h>
#include <iostream>
#include <boost/thread.hpp>
#include <mutex>
#include <queue>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <ros/ros.h>
#include <thread>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include<ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
// #include <eigen3/Eigen/Dense>


typedef pcl::PointXYZRGB PointType;

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};