#include <iostream>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZRGB PointType;

int main(int argc, char** argv)
{
    //laser是帧点云，map是要匹配的历史局部地图，fina1是align函数矫正后的帧点云，
    //fina2是transformPointCloud函数通过计算的变换矩阵矫正的点云
    //all1是cloud_laser，cloud_map，cloud_fina1一起输出的结果
    //all2是cloud_laser，cloud_map，使用四元数和位移获取矫正点云一起输出的结果
    pcl::PointCloud<PointType>::Ptr cloud_laser(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_map(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_fina1(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_fina2(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_all(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_all2(new pcl::PointCloud<PointType>);
    
    if (pcl::io::loadPCDFile<PointType>("/home/touchair/sensor_fusion_ws/src/one_liom/test_icp/data/x_5/34laser.pcd", *cloud_laser )== -1)
    {
        PCL_ERROR("Couldn't read file laser_pcd.pcd\n");
        return(-1);
    }
    if (pcl::io::loadPCDFile<PointType>("/home/touchair/sensor_fusion_ws/src/one_liom/test_icp/data/x_5/34map.pcd", *cloud_map )== -1)
    {
        PCL_ERROR("Couldn't read file map_pcd.pcd\n");
        return(-1);
    }
    
    //构建icp
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-8);
    icp.setRANSACIterations(0);

    //将A 和 B喂入 icp；；fina1是帧点云转换后的结果 
    icp.setInputSource(cloud_laser);
    icp.setInputTarget(cloud_map);
    icp.align(*cloud_fina1);
    
    //如果点云B 只是由 A进行一些简单的刚体变换得来的，icp.hasConverged()值1，如果存在形变则不为1
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
    //输出刚体变换矩阵信息,并用该变换对输入帧点云进行转换获得fina2
    std::cout << icp.getFinalTransformation() << std::endl;
    pcl::transformPointCloud(*cloud_laser, *cloud_fina2, icp.getFinalTransformation());

    //获取变换矩阵，求四元数q和位移t_vector
    Eigen::Matrix4f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    Eigen::Matrix3f r_matrix = Eigen::Matrix3f::Identity();
    for(int i=0;i<3;i++)
    {
	for(int j=0;j<3;j++)
	{
	    r_matrix(i,j) = correctionLidarFrame(i,j);
	}
    }
    Eigen::Vector3f t_vector(correctionLidarFrame(0,3),correctionLidarFrame(1,3),correctionLidarFrame(2,3));
    Eigen::Quaternionf q;
    q = Eigen::Quaternionf ( r_matrix );
    
    //准备输出的名字
    std::string str;int pcd_count=10000;
    std::stringstream ss;
    ss << pcd_count;
    ss >> str;
    std::string pcd_path = "/home/touchair/output/test_icp/";
    
    std::string map_save_path=pcd_path+str+"laser.pcd";
    pcl::io::savePCDFileBinary(map_save_path, *cloud_laser );
    
    std::string laser_save_path=pcd_path+str+"map.pcd";
    pcl::io::savePCDFileBinary(laser_save_path, *cloud_map );
    
    std::string fina1_save_path=pcd_path+str+"cloud.pcd";
    pcl::io::savePCDFileBinary(fina1_save_path, *cloud_fina1 );

    //输出all1
    for(int i=0;i<int(cloud_laser->points.size());i++)
    {
	PointType pointseed;
	pointseed.x = cloud_laser->points[i].x;
	pointseed.y = cloud_laser->points[i].y;
	pointseed.z = cloud_laser->points[i].z;
	pointseed.r = 50;
	pointseed.g = 50;
	pointseed.b = 200;
	cloud_all->points.push_back(pointseed);
    }
    
    for(int i=0;i<int(cloud_fina1->points.size());i++)
    {
	PointType pointseed;
	pointseed.x = cloud_fina1->points[i].x;
	pointseed.y = cloud_fina1->points[i].y;
	pointseed.z = cloud_fina1->points[i].z;
	pointseed.r = 200;
	pointseed.g = 50;
	pointseed.b = 50;
	cloud_all->points.push_back(pointseed);
    }
    
    for(int i=0;i<int(cloud_map->points.size());i++)
    {
	PointType pointseed;
	pointseed.x = cloud_map->points[i].x;
	pointseed.y = cloud_map->points[i].y;
	pointseed.z = cloud_map->points[i].z;
	pointseed.r = 50;
	pointseed.g = 200;
	pointseed.b = 50;
	cloud_all->points.push_back(pointseed);
    }
    
    std::string all_save_path=pcd_path+str+"all1.pcd";
    pcl::io::savePCDFileBinary(all_save_path, *cloud_all );
    
    //-----------------------------------------------------------
    //输出all2
    for(int i=0;i<int(cloud_laser->points.size());i++)
    {
	PointType pointseed;
	pointseed.x = cloud_laser->points[i].x;
	pointseed.y = cloud_laser->points[i].y;
	pointseed.z = cloud_laser->points[i].z;
	pointseed.r = 50;
	pointseed.g = 50;
	pointseed.b = 200;
	cloud_all2->points.push_back(pointseed);
    }

    //使用四元数和位移求点云矫正
    for(int i=0;i<int(cloud_laser->points.size());i++)
    {
	Eigen::Vector3f point;
	point[0]=cloud_laser->points[i].x;
	point[1]=cloud_laser->points[i].y;
	point[2]=cloud_laser->points[i].z;
	Eigen::Vector3f pointWorld = t_vector+q*point;
	PointType pointseed;
	pointseed.x = pointWorld[0];
	pointseed.y = pointWorld[1];
	pointseed.z = pointWorld[2];
	pointseed.r = 200;
	pointseed.g = 50;
	pointseed.b = 50;
	cloud_all2->points.push_back(pointseed);
    }
    
    for(int i=0;i<int(cloud_map->points.size());i++)
    {
	PointType pointseed;
	pointseed.x = cloud_map->points[i].x;
	pointseed.y = cloud_map->points[i].y;
	pointseed.z = cloud_map->points[i].z;
	pointseed.r = 50;
	pointseed.g = 200;
	pointseed.b = 50;
	cloud_all2->points.push_back(pointseed);
    }    
    
    std::string all2_save_path=pcd_path+str+"all2.pcd";
    pcl::io::savePCDFileBinary(all2_save_path, *cloud_all2 );
    
    return (0);
}