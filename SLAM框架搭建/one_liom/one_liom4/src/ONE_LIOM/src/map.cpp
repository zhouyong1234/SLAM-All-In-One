#include "one_liom.h"

//输出路径和地图
ros::Publisher pubLaserPath;
ros::Publisher pubLaserMap;

//缓存plane点和odom算出的位姿，
//这里的plane是经过odometry.cpp里的q_last_curr，t_last_curr变换过的，也就是这些点在上一帧的坐标系，这是为了和地图匹配,地图目前只有第一帧到当前帧的点，
//所以为了匹配，需要像里程计里当前帧变前一帧再匹配一样，把当前帧变到上一帧坐标系，再由ros节点接收，再通过世界到前一帧变换关系,转换到世界坐标系下
//这里的odom是里程计计算出的从里程计原点到当前的位姿变换，也就是odometry.cpp里的q_w_curr，t_w_curr
std::queue<sensor_msgs::PointCloud2ConstPtr> planeBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::mutex mBuf;

//每5帧进行一次地图降采样
int map_filter_num = 0;

double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
//世界坐标系下某个点的四元数和位移(对应rviz是蓝线)
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);
 
// wmap_T_odom * odom_T_curr = wmap_T_curr;
// transformation between odom's world and map's world frame
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);
 
//里程计坐标系下某点的四元数和位移(对应rviz是绿线)
Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);

//存放当前帧的plane点和有所有帧的plane点构成的地图点云
pcl::PointCloud<PointType>::Ptr laserCloudPlane(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMap(new pcl::PointCloud<PointType>());

//用于地图降采样
pcl::VoxelGrid<PointType> downSizeFilterMap;

//定义路径，用于保存帧的位置，发布于pubLaserPath
nav_msgs::Path laserPath;

//点面损失函数，输入的是当前帧的某点_point_o_，目标平面的中心点_point_a_，目标平面的法线_norn_，常规求ao向量在法向量上的投影
struct CURVE_PLANE_COST
{
  CURVE_PLANE_COST(Eigen::Vector3d _point_o_, Eigen::Vector3d _point_a_,Eigen::Vector3d _norn_):
		     point_o_(_point_o_),point_a_(_point_a_),norn_(_norn_){}
  template <typename T>
  bool operator()(const T* q,const T* t,T* residual)const
  {
    Eigen::Matrix<T, 3, 1> p_o_curr{T(point_o_.x()), T(point_o_.y()), T(point_o_.z())};
    Eigen::Matrix<T, 3, 1> p_a_last{T(point_a_.x()), T(point_a_.y()), T(point_a_.z())};
    Eigen::Matrix<T, 3, 1> p_norm{T(norn_.x()), T(norn_.y()), T(norn_.z())};
    Eigen::Quaternion<T> rot_q{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> rot_t{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> p_o_last;
    p_o_last=rot_q * p_o_curr + rot_t;
    residual[0]=((p_o_last - p_a_last).dot(p_norm));
    return true;
  }
  const Eigen::Vector3d point_o_,point_a_,norn_;
};

//把某点转换为地图点
void TransformToMap(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point;
    
    un_point= q_w_curr * point + t_w_curr;
 
    //输出一下
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->r = pi->r;
    po->g = pi->g;
    po->b = pi->b;
}

//ros节点句柄保存到缓存
void cloud_plane_Callhandle(const sensor_msgs::PointCloud2Ptr &plane_cloud)
{
    mBuf.lock();
    planeBuf.push(plane_cloud);
    mBuf.unlock();
}

void local_odom_Callhandle(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
    mBuf.lock();
    odometryBuf.push(laserOdometry);
    mBuf.unlock();
}

void main_thread()
{
    while(1)
    {
	//对于这种while(1)写法，加延时是为了防止出前期没数据无限运行main_thread函数导致卡死的bug，在例如aloam程序中也有对应处理(milliseconds)
	sleep(0.1);
	while(!odometryBuf.empty()&&!planeBuf.empty())
	{
	    TicToc t_map;
	    
	    mBuf.lock();
	    
	    //记录时间戳
	    double timeLaserCloudPlane = planeBuf.front()->header.stamp.toSec();
	    double timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
	    
	    if(timeLaserCloudPlane!=timeLaserOdometry)
	    {
		printf("unsync messeage! \n");
		mBuf.unlock();
		break;
	    }
	    
	    //清空上次面特征点云，并接收新的
	    laserCloudPlane->clear();
	    pcl::fromROSMsg(*planeBuf.front(), *laserCloudPlane);
	    planeBuf.pop();
	    
	    downSizeFilterMap.setInputCloud(laserCloudPlane);
	    downSizeFilterMap.filter(*laserCloudPlane);
	    
	    //接收里程计坐标系下的四元数与位移
	    q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
	    q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
	    q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
	    q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
	    t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
	    t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
	    t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
	    odometryBuf.pop();

	    mBuf.unlock();
	    //第一帧没点会进入这个if
	    if(laserCloudMap->points.size()<10) {laserCloudMap->clear(); *laserCloudMap +=  *laserCloudPlane;}
	    else
	    {
		//每5帧进行一次地图降采样
		map_filter_num++;
		if(map_filter_num%5==0)
		{
		    downSizeFilterMap.setInputCloud(laserCloudMap);
		    downSizeFilterMap.filter(*laserCloudMap);
		}
		
		//推算当前地图下的四元数和位移
		q_w_curr = q_wmap_wodom * q_wodom_curr;
		t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
		
		//把地图送入KD树
		pcl::KdTreeFLANN<PointType> kdtreePlane;
		kdtreePlane.setInputCloud(laserCloudMap);
		
		int plane_num = laserCloudPlane->points.size();
		
		for(int solve_num=0;solve_num<2;solve_num++)
		{
		    //优化问题构建
		    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
		    ceres::LocalParameterization *q_parameterization =
		    new ceres::EigenQuaternionParameterization();
		    ceres::Problem::Options problem_options;
		    ceres::Problem problem(problem_options);
		    problem.AddParameterBlock(parameters, 4, q_parameterization);
		    problem.AddParameterBlock(parameters+4, 3);
		    
		    //这个res_num是匹配上的点计数
		    int res_num=0;
		    
		    for(int i=0;i<plane_num;i++)
		    {
			//将当前帧的点转换到世界坐标系，与世界坐标系内的点找五个最近的点
			PointType pointseed;std::vector<int> pointSearchInd;std::vector<float> pointSearchSqDis;
			TransformToMap(&laserCloudPlane->points[i],&pointseed);
			kdtreePlane.nearestKSearch(pointseed, 5, pointSearchInd, pointSearchSqDis);
			
			//如果五个点里最远的那个也不超过2m，
			if (pointSearchSqDis[4] < 2.0)
			{
			    //找五个点的中心点center，并计算五点形成平面的法向量norm
			    std::vector<Eigen::Vector3d> nearCorners;
			    Eigen::Vector3d center(0, 0, 0);
			    for (int j = 0; j < 5; j++)
			    {
				    Eigen::Vector3d tmp(laserCloudMap->points[pointSearchInd[j]].x,
							laserCloudMap->points[pointSearchInd[j]].y,
							laserCloudMap->points[pointSearchInd[j]].z);
				    center = center + tmp;
				    nearCorners.push_back(tmp);
			    }
			    center = center / 5.0;
			    Eigen::Matrix<double, 5, 3> matA0;
			    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
			    for (int j = 0; j < 5; j++)
			    {
				    matA0(j, 0) = laserCloudMap->points[pointSearchInd[j]].x;
				    matA0(j, 1) = laserCloudMap->points[pointSearchInd[j]].y;
				    matA0(j, 2) = laserCloudMap->points[pointSearchInd[j]].z;
				    //printf(" pts %f %f %f \n", matA0(j, 0), matA0(j, 1), matA0(j, 2));
			    }
			    // find the norm of plane
			    //可以根据这个学习一下https://www.cnblogs.com/wangxiaoyong/p/8977343.html
			    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
			    norm.normalize();

			    //将五个点和中心点形成向量，向量与法向量求点乘，如果大于0.1那么后面就不把这组点放入优化了
			    bool planeValid = true;
			    for (int j = 0; j < 5; j++)
			    {
				    Eigen::Vector3d vector_temp(laserCloudMap->points[pointSearchInd[j]].x-center.x(),
								laserCloudMap->points[pointSearchInd[j]].y-center.y(),
								laserCloudMap->points[pointSearchInd[j]].z-center.z());
			      
				    if (fabs(norm(0) * vector_temp.x() +norm(1) * vector_temp.y() +norm(2) * vector_temp.z()) > 0.1)
				    {
					    planeValid = false;
					    break;
				    }
			    }
			    
			    //当前点curr_point，放入优化
			    Eigen::Vector3d curr_point(pointseed.x, pointseed.y, pointseed.z);
			    if (planeValid)
			    {
				    Eigen::Vector3d curr_point_o(laserCloudPlane->points[i].x,laserCloudPlane->points[i].y,laserCloudPlane->points[i].z);
				    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CURVE_PLANE_COST,1,4,3>
							    (new CURVE_PLANE_COST(curr_point_o,center,norm)),loss_function,parameters,parameters+4);
				    res_num++;
			    }
			}
		    }
		    
		    //如果优化的组大于40，进行优化
		    if(res_num>40)
		    {
			ceres::Solver::Options options;
			options.linear_solver_type = ceres::DENSE_QR;
			//迭代数
			options.max_num_iterations = 4;
			//进度是否发到STDOUT
			options.minimizer_progress_to_stdout = false;
			options.gradient_check_relative_precision = 1e-4;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
		    }
		    
		    //优化后更迭地图和里程计之间的变换
		    q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
		    t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
		}
		
		//保存当前点
		for(int i=0;i<plane_num;i++)
		{
		    PointType pointseed;
		    TransformToMap(&laserCloudPlane->points[i],&pointseed);
		    laserCloudMap->points.push_back(pointseed);
		}	
	    }    
	
	    //输出轨迹
	    nav_msgs::Odometry laserOdometry;
	    laserOdometry.header.frame_id = "map";
	    laserOdometry.child_frame_id = "map_child";
	    laserOdometry.header.stamp = ros::Time().fromSec(timeLaserCloudPlane);
	    laserOdometry.pose.pose.orientation.x = q_w_curr.x();
	    laserOdometry.pose.pose.orientation.y = q_w_curr.y();
	    laserOdometry.pose.pose.orientation.z = q_w_curr.z();
	    laserOdometry.pose.pose.orientation.w = q_w_curr.w();
	    laserOdometry.pose.pose.position.x = t_w_curr.x();
	    laserOdometry.pose.pose.position.y = t_w_curr.y();
	    laserOdometry.pose.pose.position.z = t_w_curr.z();

	    geometry_msgs::PoseStamped laserPose;
	    laserPose.header = laserOdometry.header;
	    laserPose.pose = laserOdometry.pose.pose;
	    laserPath.header.stamp = laserOdometry.header.stamp;
	    laserPath.poses.push_back(laserPose);
	    laserPath.header.frame_id = "map";
	    pubLaserPath.publish(laserPath); 
	    
	    //输出点云显示
	    pcl::PointCloud<PointType>::Ptr laserCloudMap_Out(new pcl::PointCloud<PointType>());
	    long map_num = laserCloudMap->points.size();
	    for(long i=0;i<map_num;i++)
	    {
		PointType pointseed;
		pointseed.x = laserCloudMap->points[i].x;
		pointseed.y = laserCloudMap->points[i].y;
		pointseed.z = laserCloudMap->points[i].z;
		laserCloudMap_Out->points.push_back(pointseed);
	    }
	    
	    if(map_filter_num%20==0&&map_filter_num!=0)
	    {
		sensor_msgs::PointCloud2 laserCloudMapMsg;
		pcl::toROSMsg(*laserCloudMap_Out, laserCloudMapMsg);
		laserCloudMapMsg.header.stamp = ros::Time().fromSec(timeLaserCloudPlane);
		laserCloudMapMsg.header.frame_id = "map";
		pubLaserMap.publish(laserCloudMapMsg);
	    }
	    printf("map prepare time %f ms\n", t_map.toc());
	}
    }
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;
    
    downSizeFilterMap.setLeafSize(0.4, 0.4, 0.4);
    
    ros::Subscriber LaserCloudmap = n.subscribe("/velodyne_cloud_map", 100, cloud_plane_Callhandle);
    ros::Subscriber LaserLocalOdom = n.subscribe("/laser_odom_to_init", 100, local_odom_Callhandle);
    
    //输出map后的轨迹与位姿
    pubLaserPath = n.advertise<nav_msgs::Path>("/laser_map_path", 100);
    pubLaserMap  = n.advertise<sensor_msgs::PointCloud2>("/laser_map_cloud",100);
    
    boost::thread server(main_thread);
    
    ros::spin();
    return 0;
}