#include "one_liom.h"

//这里使用pcl的rgb并不是颜色，只是拿来储存信息
//r是是否需要可以加入特征点云，初始为1，表示可以选，如果某点已经被选，那么该点前后各5个点会被设为2，不可选
//g是曲率，认为是plane point会从小到大添加为1-4
//b是scan的线号0-15
typedef pcl::PointXYZRGB PointType;

//输出全部点，平面点，地面点
ros::Publisher pubLaserCloudall;
ros::Publisher pubLaserCloudplane;
ros::Publisher pubLaserCloudground;

//定义路径，用于保存帧的位置，发布于pubLaserPath
nav_msgs::Path laserPath;

//储存全部点，平面点，地面点
pcl::PointCloud<PointType>::Ptr laserCloudall(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudplane(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudground(new pcl::PointCloud<PointType>());

//点的序号，初始为1-laserCloudall.size()顺序排列，后续进行了排序操作
int cloudSortInd[400000];
//曲率存放
double cloudcurv[400000];
//用于排序，输入i和j，获取对应点的曲率对比，在后续使用中，会将cloudSortInd的某一区间指代的点曲率从小到大排序
//不太理解去查sort函数
bool comp (int i,int j) { return (cloudcurv[i]<cloudcurv[j]); }

void cloud_Callhandle(const sensor_msgs::PointCloud2 ros_cloud)
{
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(ros_cloud, laserCloudIn);
    int cloudSize = laserCloudIn.points.size();
    int count = cloudSize;
    PointType point;
    
    //存储每线点云
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(16);

    //清空内存
    laserCloudall->clear();
    laserCloudplane->clear();
    laserCloudground->clear();
	
    //判定各点的线数,按线保存在laserCloudScans里
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
	point.r=1;point.g=1;
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

	scanID = int((angle + 15) / 2 + 0.5);
	if (scanID > (15) || scanID < 0)
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
	    }
	}
    }
    
    //用于比较曲率大小
    long ind_count = 0;
    //第一个元素是0，后面记录每线的终止位置
    int cloudScanEndInd[17];
    cloudScanEndInd[0]=0;
    
    //计算曲率
    for(int i=0;i<16;i++)
    {
	for(int j=5;j<int(laserCloudScans[i].size())-5;j++)
	{
	    float diffX = laserCloudScans[i].points[j - 5].x + laserCloudScans[i].points[j - 4].x + laserCloudScans[i].points[j - 3].x + laserCloudScans[i].points[j - 2].x + laserCloudScans[i].points[j - 1].x - 10 * laserCloudScans[i].points[j].x + laserCloudScans[i].points[j + 1].x + laserCloudScans[i].points[j + 2].x + laserCloudScans[i].points[j + 3].x + laserCloudScans[i].points[j + 4].x + laserCloudScans[i].points[j + 5].x;
	    float diffY = laserCloudScans[i].points[j - 5].y + laserCloudScans[i].points[j - 4].y + laserCloudScans[i].points[j - 3].y + laserCloudScans[i].points[j - 2].y + laserCloudScans[i].points[j - 1].y - 10 * laserCloudScans[i].points[j].y + laserCloudScans[i].points[j + 1].y + laserCloudScans[i].points[j + 2].y + laserCloudScans[i].points[j + 3].y + laserCloudScans[i].points[j + 4].y + laserCloudScans[i].points[j + 5].y;
	    float diffZ = laserCloudScans[i].points[j - 5].z + laserCloudScans[i].points[j - 4].z + laserCloudScans[i].points[j - 3].z + laserCloudScans[i].points[j - 2].z + laserCloudScans[i].points[j - 1].z - 10 * laserCloudScans[i].points[j].z + laserCloudScans[i].points[j + 1].z + laserCloudScans[i].points[j + 2].z + laserCloudScans[i].points[j + 3].z + laserCloudScans[i].points[j + 4].z + laserCloudScans[i].points[j + 5].z;
	    cloudcurv[ind_count]=(diffX * diffX + diffY * diffY + diffZ * diffZ);
	    laserCloudall->points.push_back(laserCloudScans[i].points[j]);
	    ind_count++;
	    cloudSortInd[ind_count]=ind_count;
	}
	cloudScanEndInd[i+1] = laserCloudall->points.size();
    }
    
    int laser_num = laserCloudall->points.size();
    
    //利用cloudScanEndInd将每条线分成六块，每块使用sort排序，并从最平缓的4个点录入laserCloudplane，录入后对周围点作不选取的标志
    for(int i=0;i<16;i++)
    {
	//每条线起止
	int start_num = cloudScanEndInd[i];
	int end_num = cloudScanEndInd[i+1];
	//分六块
	for(int j=0;j<6;j++)
	{
	    int start_num_temp = start_num+ ((end_num-start_num)/6)*j;
	    int end_num_temp = start_num+ ((end_num-start_num)/6)*(j+1);
	    //块内排序cloudSortInd会从1,2,3,4这种顺序变成乱序，乱序后指代的点曲率从小到大排列
	    std::sort (cloudSortInd + start_num_temp, cloudSortInd + end_num_temp, comp);
	    //计数4个
	    int plane_num=0;
	    //k要在区间内，点要小于5个
	    for(int k=start_num_temp;k<end_num_temp&&plane_num<6;k++)
	    {
		//根据cloudSortInd取点序号
		long ind = cloudSortInd[k];
		//可选点+曲率小就要
		if(laserCloudall->points[ind].r==1&&cloudcurv[ind]<0.1)
		{
		    plane_num++;
		    laserCloudall->points[ind].g=plane_num;
		    laserCloudplane->push_back(laserCloudall->points[ind]);
		    //临近点变成不可选
		    for(int m=1;ind+m<laser_num&&m<=5;m++)
		    {
			laserCloudall->points[ind+m].r=2;
		    }
		    for(int m=1;ind-m>0&&m<=5;m++)
		    {
			laserCloudall->points[ind-m].r=2;
		    }
		}
	    }
	}
    }
    
    //地面，棱，平面，全部点云输出
    sensor_msgs::PointCloud2 laserCloudgroundMsg;
    pcl::toROSMsg(*laserCloudground, laserCloudgroundMsg);
    laserCloudgroundMsg.header.stamp = ros_cloud.header.stamp;
    laserCloudgroundMsg.header.frame_id = "map_child";
    pubLaserCloudground.publish(laserCloudgroundMsg);  
    
    sensor_msgs::PointCloud2 laserCloudplaneMsg;
    pcl::toROSMsg(*laserCloudplane, laserCloudplaneMsg);
    laserCloudplaneMsg.header.stamp = ros_cloud.header.stamp;
    laserCloudplaneMsg.header.frame_id = "map_child";
    pubLaserCloudplane.publish(laserCloudplaneMsg); 
    
    sensor_msgs::PointCloud2 laserCloudallMsg;
    pcl::toROSMsg(*laserCloudall, laserCloudallMsg);
    laserCloudallMsg.header.stamp = ros_cloud.header.stamp;
    laserCloudallMsg.header.frame_id = "map_child";
    pubLaserCloudall.publish(laserCloudallMsg);
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "rosbagregist");
    ros::NodeHandle n;
    ros::Subscriber cloud_sub = n.subscribe("/velodyne_points", 100, cloud_Callhandle);
    
    pubLaserCloudground = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_ground", 100);
    pubLaserCloudplane = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_plane", 100);
    pubLaserCloudall = n.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_all", 100);
    ros::spin();
    return 0;
}