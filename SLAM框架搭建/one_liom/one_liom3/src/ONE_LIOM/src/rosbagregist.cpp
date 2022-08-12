#include <cmath>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//这里使用pcl的rgb并不是颜色，只是拿来储存信息
//r是是否为地面，1：非地面 2：地面
//g是曲率，小于一定值（比如0.1）认为是plane point
//b是scan的线号0-15
typedef pcl::PointXYZRGB PointType;

ros::Publisher pubLaserCloudground;
ros::Publisher pubLaserCloudedge;
ros::Publisher pubLaserCloudplane;
ros::Publisher pubLaserCloudall_01;

double tf_x = 0;
clock_t  clock_start, clock_end;

void cloud_Callhandle(const sensor_msgs::PointCloud2 ros_cloud)
{
    clock_start=clock();
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(ros_cloud, laserCloudIn);
    int cloudSize = laserCloudIn.points.size();
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(32);
    pcl::PointCloud<PointType>::Ptr laserCloudground(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserCloudedge(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserCloudplane(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserCloudall(new pcl::PointCloud<PointType>());
	
    //判定各点的线数
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
	    point.r=1;
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        // scanID = int((angle + 15) / 2 + 0.5);
        scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
        if (scanID > (31) || scanID < 0)
        {
            count--;
            continue;
        }
        point.b=scanID;
        laserCloudScans[scanID].push_back(point);
    }
    
    //地面点获取
    for(int i=0;i<5;i++)
    {
	for(int j=0;j<int(laserCloudScans[i].size())&&j<int(laserCloudScans[i+1].size());j++)
	{
	    float diffX,diffY,diffZ,angle;
	    diffX = laserCloudScans[i+1].points[j].x - laserCloudScans[i].points[j].x;
	    diffY = laserCloudScans[i+1].points[j].y - laserCloudScans[i].points[j].y;
	    diffZ = laserCloudScans[i+1].points[j].z - laserCloudScans[i].points[j].z;
	    angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
	    if(abs(angle)<10) 
	    {
		laserCloudground->push_back(laserCloudScans[i+1].points[j]);
		laserCloudground->push_back(laserCloudScans[i].points[j]);
		laserCloudScans[i+1].points[j].r=2;
		laserCloudScans[i].points[j].r=2;
	    }
	}
    }
    
    //计算曲率
    for(int i=0;i<32;i++)
    {
	for(int j=3;j<int(laserCloudScans[i].size())-3;j++)
	{
	    float diffX = laserCloudScans[i].points[j - 3].x + laserCloudScans[i].points[j - 2].x + laserCloudScans[i].points[j - 1].x - 6 * laserCloudScans[i].points[j].x + laserCloudScans[i].points[j + 1].x + laserCloudScans[i].points[j + 2].x + laserCloudScans[i].points[j + 3].x;
	    float diffY = laserCloudScans[i].points[j - 3].y + laserCloudScans[i].points[j - 2].y + laserCloudScans[i].points[j - 1].y - 6 * laserCloudScans[i].points[j].y + laserCloudScans[i].points[j + 1].y + laserCloudScans[i].points[j + 2].y + laserCloudScans[i].points[j + 3].y;
	    float diffZ = laserCloudScans[i].points[j - 3].z + laserCloudScans[i].points[j - 2].z + laserCloudScans[i].points[j - 1].z - 6 * laserCloudScans[i].points[j].z + laserCloudScans[i].points[j + 1].z + laserCloudScans[i].points[j + 2].z + laserCloudScans[i].points[j + 3].z;
	    laserCloudScans[i].points[j].g=double(diffX * diffX + diffY * diffY + diffZ * diffZ);
	}
    }
    
    for (int i = 0; i < 32; i++)
    { 
        *laserCloudall += laserCloudScans[i];
    }

    // for(int i = 0; i < 32; i++)
    // {
    //     for(int j = 5; j < int(laserCloudScans[i].size()) - 5; j++)
    //     {
    //         if(laserCloudScans[i].points[j].r == 1)
    //         {
    //             if(laserCloudScans[i].points[j].g>0.2) 
    //             {
    //                 laserCloudedge->push_back(laserCloudScans[i].points[j]);
    //                 j=j+5;
    //             }
    //             if(laserCloudScans[i].points[j].g<0.05) 
    //             {
    //                 laserCloudplane->push_back(laserCloudScans[i].points[j]);
    //                 j=j+5;
    //             }
    //         }
    //     }
    // }
    
    long all_points_num = laserCloudall->points.size();
    for(int i=0;i<62;i++)
    {
        long start_num = i*all_points_num/62;
        long end_num = (i+1)*all_points_num/62;
        for(long j=start_num;j<end_num;j++)
        {
            if(laserCloudall->points[j].r==1)
            {
            if(laserCloudall->points[j].g>0.2&&(int(laserCloudedge->points.size())<4*(i+1))) 
            {
                laserCloudedge->push_back(laserCloudall->points[j]);
                j=j+5;
            }
            if(laserCloudall->points[j].g<0.05&&(int(laserCloudplane->points.size())<20*(i+1))) 
            {
                laserCloudplane->push_back(laserCloudall->points[j]);
                j=j+5;
            }
            }
            if((int(laserCloudplane->points.size())==20*(i+1))&&(int(laserCloudedge->points.size())==4*(i+1)))
            {
                j=end_num;
            }
        }
    }
    
    //地面，棱，平面，全部点云输出
    sensor_msgs::PointCloud2 laserCloudgroundMsg;
    pcl::toROSMsg(*laserCloudground, laserCloudgroundMsg);
    laserCloudgroundMsg.header.stamp = ros_cloud.header.stamp;
    laserCloudgroundMsg.header.frame_id = "map_child";
    pubLaserCloudground.publish(laserCloudgroundMsg);
    
    sensor_msgs::PointCloud2 laserCloudedgeMsg;
    pcl::toROSMsg(*laserCloudedge, laserCloudedgeMsg);
    laserCloudedgeMsg.header.stamp = ros_cloud.header.stamp;
    laserCloudedgeMsg.header.frame_id = "map_child";
    pubLaserCloudedge.publish(laserCloudedgeMsg);   
    
    sensor_msgs::PointCloud2 laserCloudplaneMsg;
    pcl::toROSMsg(*laserCloudplane, laserCloudplaneMsg);
    laserCloudplaneMsg.header.stamp = ros_cloud.header.stamp;
    laserCloudplaneMsg.header.frame_id = "map_child";
    pubLaserCloudplane.publish(laserCloudplaneMsg); 
    
    sensor_msgs::PointCloud2 laserCloudallMsg;
    pcl::toROSMsg(*laserCloudall, laserCloudallMsg);
    laserCloudallMsg.header.stamp = ros_cloud.header.stamp;
    laserCloudallMsg.header.frame_id = "map_child";
    pubLaserCloudall_01.publish(laserCloudallMsg);
    
    clock_end = clock();
    std::cout<<"scan time:"<<double(clock_end-clock_start)/ CLOCKS_PER_SEC<<std::endl;
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "rosbagregist");
    ros::NodeHandle n;
    ros::Subscriber cloud_sub = n.subscribe("/velodyne_points", 100, cloud_Callhandle);
    pubLaserCloudground = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_ground", 100);
    pubLaserCloudedge = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_edge", 100);
    pubLaserCloudplane = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_plane", 100);
    pubLaserCloudall_01 = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_all_01", 100);
    ros::spin();
    return 0;
}
