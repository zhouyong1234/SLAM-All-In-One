#include <cmath>
#include <vector>
#include <string>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointType;

ros::Publisher pubLaserCloudground;
ros::Publisher pubLaserCloudedge;
ros::Publisher pubLaserCloudplane;

void cloud_Callhandle(const sensor_msgs::PointCloud2 ros_cloud)
{
    clock_t  clock_start,clock_end;
    clock_start=clock();
    //输出世界参考系的坐标
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(0,0,0));
    q.setW(1);
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros_cloud.header.stamp, "map", "map_child"));
    
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(ros_cloud, laserCloudIn);
    int cloudSize = laserCloudIn.points.size();
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(32);
    pcl::PointCloud<PointType>::Ptr laserCloudground(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserCloudedge(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserCloudplane(new pcl::PointCloud<PointType>());
    
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
    
    //根据曲率获取edge点和plane点
    for(int i=0;i<32;i++)
    {
	for(int j=5;j<int(laserCloudScans[i].size())-5;j++)
	{
	    if(laserCloudScans[i].points[j].r==1)
	    {
            if(laserCloudScans[i].points[j].g>0.2) 
            {
                laserCloudedge->push_back(laserCloudScans[i].points[j]);
                j=j+5;
            }
            if(laserCloudScans[i].points[j].g<0.1) 
            {
                laserCloudplane->push_back(laserCloudScans[i].points[j]);
                j=j+5;
            }
	    }
	}
    }
    
    sensor_msgs::PointCloud2 laserCloudgroundMsg;
    pcl::toROSMsg(*laserCloudground, laserCloudgroundMsg);
    laserCloudgroundMsg.header.stamp = ros_cloud.header.stamp;
    laserCloudgroundMsg.header.frame_id = "map";
    pubLaserCloudground.publish(laserCloudgroundMsg);
    
    sensor_msgs::PointCloud2 laserCloudedgeMsg;
    pcl::toROSMsg(*laserCloudedge, laserCloudedgeMsg);
    laserCloudedgeMsg.header.stamp = ros_cloud.header.stamp;
    laserCloudedgeMsg.header.frame_id = "map";
    pubLaserCloudedge.publish(laserCloudedgeMsg);   
    
    sensor_msgs::PointCloud2 laserCloudplaneMsg;
    pcl::toROSMsg(*laserCloudplane, laserCloudplaneMsg);
    laserCloudplaneMsg.header.stamp = ros_cloud.header.stamp;
    laserCloudplaneMsg.header.frame_id = "map";
    pubLaserCloudplane.publish(laserCloudplaneMsg); 
    
    clock_end = clock();
    std::cout<<"time:"<<double(clock_end-clock_start)/ CLOCKS_PER_SEC<<std::endl;
    
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "rosbagregist");
    ros::NodeHandle n;
    ros::Subscriber cloud_sub = n.subscribe("/velodyne_points", 100, cloud_Callhandle);
    pubLaserCloudground = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_ground", 100);
    pubLaserCloudedge = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_edge", 100);
    pubLaserCloudplane = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_plane", 100);
    ros::spin();
    return 0;
}
