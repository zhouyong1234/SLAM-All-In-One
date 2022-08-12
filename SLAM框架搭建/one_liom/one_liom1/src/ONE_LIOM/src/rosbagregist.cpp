#include <cmath>
#include <vector>
#include <string>
//ros用
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//世界统一参考系用
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZI PointType;

int N_SCANS=16;
ros::Publisher pubLaserCloudup;
ros::Publisher pubLaserClouddown;

void cloud_Callhandle(const sensor_msgs::PointCloud2 ros_cloud)
{
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
    
    //接收点云转换为PCL格式
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(ros_cloud, laserCloudIn);
    //计数用于循环
    int cloudSize = laserCloudIn.points.size();
    int count = cloudSize;
    PointType point;
    
    //判定各点的线数，按线数保存
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

	    //仰角
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

         if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);// + 0.5 是用于四舍五入   因为 int 只会保留整数部分  如 int(3.7) = 3  
	                                         //  /2是每两个scan之间的间隔为2度，+15是过滤垂直上为[-,15,15]范围内
            if (scanID > (N_SCANS - 1) || scanID < 0)//scanID是0-15号
            {//该点不符合要求，舍弃
                count--;
                continue;
            }
        }
        
         // 下面两种为32线与64线
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }

        laserCloudScans[scanID].push_back(point);
    }
    
    //按1-8,9-16线保存
    pcl::PointCloud<PointType>::Ptr laserCloudup(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserClouddown(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS/2; i++)
    { 
        *laserCloudup += laserCloudScans[i];
    }
    for (int i = N_SCANS/2; i < N_SCANS; i++)
    { 
        *laserClouddown += laserCloudScans[i];
    }
    
    //pcl转ros输出格式，map是统一坐标系
    sensor_msgs::PointCloud2 laserCloudupOutMsg;
    pcl::toROSMsg(*laserCloudup, laserCloudupOutMsg);
    laserCloudupOutMsg.header.stamp = ros_cloud.header.stamp;
    laserCloudupOutMsg.header.frame_id = "map";
    pubLaserCloudup.publish(laserCloudupOutMsg);
    
    sensor_msgs::PointCloud2 laserClouddownOutMsg;
    pcl::toROSMsg(*laserClouddown, laserClouddownOutMsg);
    laserClouddownOutMsg.header.stamp = ros_cloud.header.stamp;
    laserClouddownOutMsg.header.frame_id = "map";
    pubLaserClouddown.publish(laserClouddownOutMsg);
}

//ros初始化，建立节点，接收topic为/velodyne_points的点云数据，发送
//topic为/velodyne_cloud_up，down的点云，循环
int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "rosbagregist");
    ros::NodeHandle n;
    ros::Subscriber cloud_sub = n.subscribe("/ns1/velodyne_points", 100, cloud_Callhandle);
    pubLaserCloudup = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_up", 100);
    pubLaserClouddown = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_down", 100);
    ros::spin();
    return 0;
}

