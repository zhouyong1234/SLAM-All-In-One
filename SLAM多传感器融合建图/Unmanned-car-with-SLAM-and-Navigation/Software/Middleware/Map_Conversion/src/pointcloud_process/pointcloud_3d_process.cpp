/*该类的功能主要是点云的三维栅格处理
 edited by goldqiu -2022-04-9
*/
#include "map_conversion/pointcloud_process/pointcloud_3d_process.hpp"
#include "glog/logging.h"

typedef GridNode* GridNodePtr;
GridNodePtr *** GridNodeMap;

namespace map_conversion {
Pointcloud3dProcess::Pointcloud3dProcess(YAML::Node& config_node) {
   //三维栅格初始化
   _map_lower << -_y_size/2.0, -_y_size/2.0, 0.0;
   _map_upper << +_x_size/2.0, +_y_size/2.0, _z_size;
	
	 _inv_resolution = 1.0 / _resolution;
	 int _max_x_id = (int)(_x_size * _inv_resolution);
	 int _max_y_id = (int)(_y_size * _inv_resolution);
	 int _max_z_id = (int)(_z_size * _inv_resolution);
							   
   GLX_SIZE = _max_x_id;
   GLY_SIZE = _max_y_id;
   GLZ_SIZE = _max_z_id;
   GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
   GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;
   initGridMap(_resolution, _map_lower, _map_upper);//要初始化地图 
}
Eigen::Vector3i Pointcloud3dProcess::coord2gridIndex(const Eigen::Vector3d pt) 
{
    Eigen::Vector3i idx;
    idx <<  std::min( std::max( int( (pt(0) - gl_xl) * _inv_resolution), 0), GLX_SIZE - 1),
            std::min( std::max( int( (pt(1) - gl_yl) * _inv_resolution), 0), GLY_SIZE - 1),
            std::min( std::max( int( (pt(2) - gl_zl) * _inv_resolution), 0), GLZ_SIZE - 1);      		    
  
    return idx;
};

bool Pointcloud3dProcess::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return false;

    tmp_id_x = static_cast<int>( (coord_x - gl_xl) * _inv_resolution);
    tmp_id_y = static_cast<int>( (coord_y - gl_yl) * _inv_resolution);
    tmp_id_z = static_cast<int>( (coord_z - gl_zl) * _inv_resolution);      

    if(data[tmp_id_x * GLYZ_SIZE + tmp_id_y * GLZ_SIZE + tmp_id_z] == 0){
        data[tmp_id_x * GLYZ_SIZE + tmp_id_y * GLZ_SIZE + tmp_id_z] = 1;
        return true;
    }
    else{
        return false;
    }
}

Eigen::Vector3d Pointcloud3dProcess::gridIndex2coord(const Eigen::Vector3i index) 
{
    Eigen::Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * _resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * _resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * _resolution + gl_zl;

    return pt;
};

void Pointcloud3dProcess::initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    double resolution = _resolution;
    double inv_resolution = 1.0 / _resolution;    

    data    = new uint8_t[GLXYZ_SIZE];

    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++)
            {
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
                GridNodeMap[i][j][k]->occupancy = & data[i * GLYZ_SIZE + j * GLZ_SIZE + k];
            }
        }
    }
}
void Pointcloud3dProcess::pointcloud_to_3dGridMap(CloudData& cloud_data,CloudData& cloud_inf)
{
    pcl::PointXYZ pt, pt_inf;
    int inf_step   = round(_cloud_margin * _inv_resolution); 
    //这里涉及膨胀地图的大小，resolution 为一个栅格大小，
	//单位为m,设置为[0.15 0.30]之间，越大地图越稀疏，_inv_resolution,为resolution的倒数                                                 
    int inf_step_z = max(1, inf_step / 2);
    for (int idx = 0; idx < (int)cloud_data.cloud_ptr->points.size(); idx++)
    {    
        pt = cloud_data.cloud_ptr->points[idx];        
        for(int x = -inf_step ; x <= inf_step; x ++ )
      {
          for(int y = -inf_step ; y <= inf_step; y ++ )
        {
            for(int z = -inf_step_z; z <= inf_step_z; z ++ )
           {
              double inf_x = pt.x + x * _resolution;
              double inf_y = pt.y + y * _resolution;
              double inf_z = pt.z + z * _resolution;

              if(isnan(inf_x) || isnan(inf_y) || isnan(inf_z)) continue;

              Vector3d vec_inf(inf_x, inf_y, inf_z);
              Vector3i idx_inf = coord2gridIndex(vec_inf);     

            // set in obstacle points
              bool flag_newly_occupied = setObs(inf_x, inf_y, inf_z); 
                          
            // rounding for visualizing the grid map
              if(flag_newly_occupied) 
            {
                Vector3d coor_round = gridIndex2coord(idx_inf); 
                pt_inf.x = coor_round(0);
                pt_inf.y = coor_round(1);
                pt_inf.z = coor_round(2);
                cloud_inf.cloud_ptr->points.push_back(pt_inf);
            }
            }
        }
     }
    }
        cloud_inf.cloud_ptr->width    = cloud_inf.cloud_ptr->points.size();
        cloud_inf.cloud_ptr->height   = 1;
        cloud_inf.cloud_ptr->is_dense = true;
}
} // namespace data_input