#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZRGB PointType;

int score_0_3_num=0;
int score_0_6_num=0;
int score_1_0_num=0;
int read_num=0;

int main(int argc, char** argv)
{
    std::string pcd_path = "/home/touchair/sensor_fusion_ws/src/one_liom/test_icp/data/x_5/";
    for(int i=32;i<40;i++)
    {
	std::string str;int pcd_count=i;
	std::stringstream ss;
	ss << pcd_count;
	ss >> str;
	std::string laser_read_path=pcd_path+str+"laser.pcd";
	std::string map_read_path=pcd_path+str+"map.pcd";
	
	pcl::PointCloud<PointType>::Ptr cloud_laser(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_map(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_fina(new pcl::PointCloud<PointType>);
	
	if (pcl::io::loadPCDFile<PointType>(map_read_path, *cloud_map )== -1)
	{
	    continue;
	}
	else if (pcl::io::loadPCDFile<PointType>(laser_read_path, *cloud_laser )== -1)
	{
	    continue;
	}
	else
	{
	    //构建icp
	    pcl::IterativeClosestPoint<PointType, PointType> icp;
	    icp.setMaxCorrespondenceDistance(100);
	    icp.setMaximumIterations(100);
	    icp.setTransformationEpsilon(1e-6);
	    icp.setEuclideanFitnessEpsilon(1e-6);
	    icp.setRANSACIterations(0);

	    //将A 和 B喂入 icp
	    icp.setInputSource(cloud_laser);
	    icp.setInputTarget(cloud_map);
	    icp.align(*cloud_fina);
	    
	    double score = icp.getFitnessScore();
	    if(score<0.3) {score_0_3_num++;std::cout<<"num "<<i<<" 's icp  score is 0.3 "<<std::endl;}
	    else if(score<0.6) {score_0_6_num++;std::cout<<"num "<<i<<" 's icp  score is 0.6 "<<std::endl;}
	    else if(score<1.0) {score_1_0_num++;std::cout<<"num "<<i<<" 's icp  score is 1.0 "<<std::endl;}
	}
	std::cout<<"-----------------------------"<<std::endl;
	std::cout<<"score is"<<score_0_3_num<<" "<<score_0_6_num<<" "<<score_1_0_num<<" "<<std::endl;
    }
    return (0);
}