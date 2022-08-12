/*该类的功能主要是点云的三维栅格处理
 edited by goldqiu -2022-04-9
*/
#ifndef MAP_CONVERSION_POINTCLOUD_3D_PROCESS_HPP_
#define MAP_CONVERSION_POINTCLOUD_3D_PROCESS_HPP_
#include "map_conversion/ros_topic_interface/cloud_data.hpp"
#include <yaml-cpp/yaml.h>
#include "map_conversion/utility.hpp"

namespace map_conversion {
class Pointcloud3dProcess {
  public:
    Pointcloud3dProcess(YAML::Node& config_node);
    Pointcloud3dProcess() = default;
    
    //以下是三维栅格化的成员方法
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d pt); 
    bool setObs(const double coord_x, const double coord_y, const double coord_z);
    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i index);
    void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u);
    void pointcloud_to_3dGridMap(CloudData& cloud_data,CloudData& cloud_inf);

    double  _resolution =  0.15 ;
    double  _cloud_margin = 0.3;
    double _inv_resolution;
    double _x_size = 120.0; //要生成的地图大小 20*20*3 单位为m
    double _y_size = 120.0;
    double _z_size = 7.0 ;

    double gl_xl, gl_yl, gl_zl;
    double gl_xu, gl_yu, gl_zu;
    int GLX_SIZE;
    int GLY_SIZE;
    int GLZ_SIZE;
    int GLYZ_SIZE;
    int GLXYZ_SIZE;
    int tmp_id_x, tmp_id_y, tmp_id_z;
    uint8_t * data;
    Eigen::Vector3d _map_lower,_map_upper;
    
  private:

  private:

};
}

#endif