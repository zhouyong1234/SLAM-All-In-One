/*该类的功能主要是点云的二维栅格处理
 edited by goldqiu -2022-04-9
*/
#ifndef MAP_CONVERSION_POINTCLOUD_2D_PROCESS_HPP_
#define MAP_CONVERSION_POINTCLOUD_2D_PROCESS_HPP_
#include "map_conversion/ros_topic_interface/cloud_data.hpp"
#include <yaml-cpp/yaml.h>
#include "map_conversion/utility.hpp"

namespace map_conversion {
class Pointcloud2dProcess {
  public:
    Pointcloud2dProcess(YAML::Node& config_node);
    Pointcloud2dProcess() = default;
    
    void find_Z_value(CloudData& cloud_data); //计算点云中Z轴的最大和最小值
    int global_map_init(void); //初始化全局地图
    //直通滤波器
    void PassThroughFilter(CloudData& pcd_cloud,CloudData& cloud_after_PassThrough, const bool &flag_in);
    //将三维点云转换为三维栅格
    void Pointcloud_to_2d_grid(const CloudData& pcd_cloud, nav_msgs::OccupancyGrid& msg);

    double max_z;
    double min_z;
    std::string Global_map_file;
    std::string local_cloud_topic;

    CloudData global_map_data;
    CloudData global_map_after_filter;


    double grid_x = 0.1;
    double grid_y = 0.1;
    double grid_z = 0.1;

    double map_resolution = 0.05;
    double thre_radius = 0.5; 
    double global_z_adjust;
    double local_z_adjust;

  private:

  private:

};
}

#endif