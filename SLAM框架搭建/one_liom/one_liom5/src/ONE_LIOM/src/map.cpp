#include "one_liom.h"

//输出路径和地图
ros::Publisher pubLaserPath;
ros::Publisher pubLaserMap;

//缓存plane点和odom算出的位姿，
//这里的plane是经过odometry.cpp里的q_last_curr，t_last_curr变换过的，也就是这些点在上一帧的坐标系，这是为了和地图匹配,地图目前只有第一帧到当前帧的点，
//所以为了匹配，需要像里程计里当前帧变前一帧再匹配一样，把当前帧变到上一帧坐标系，再由ros节点接收，再通过世界到前一帧变换关系,转换到世界坐标系下
//这里的odom是里程计计算出的从里程计原点到当前的位姿变换，也就是odometry.cpp里的q_w_curr，t_w_curr
std::queue<sensor_msgs::PointCloud2ConstPtr> cloudBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> planeBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::mutex mBuf;

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

//存放当前帧的全部点，plane点，位置临近帧的plane点构成的地图点云，我们使用plane点进行帧和局部地图（包括前200帧和历史位置临近的帧）的匹配，
//使用当前帧全部点和历史临近（也就是没有前200帧的位置临近帧）帧的全部点进行闭环检测
pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudPlane(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMap(new pcl::PointCloud<PointType>());

//用于icp的当前帧和历史帧(不能在前两百帧产生)的全部点
pcl::PointCloud<PointType>::Ptr laserCloud_now_out(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloud_map_out(new pcl::PointCloud<PointType>());
//每二十帧降采样保存一次局部地图，会保存为pcd文件，在icp时根据位置提取出来，之所以要另存是因为保存在内存太大了
pcl::PointCloud<PointType>::Ptr laserCloud_local_map(new pcl::PointCloud<PointType>());

//由于plane点较少（每帧不到400点），我们保存全部plane点在内存里，即使10W点也就只有300MB左右，放内存可以承担
//laserCloudMap_Ind记录每帧plane点保存的终止时点数,暂定10万帧，比如laserCloudMap_Ind[200]=10086,就是第二百帧结束时有10086个点配合laserCloudMap_Ind[199]=10000，就可以精确定位第N帧的点提取出来。
//temp_laserCloudMap_Ind是指当前第几帧，会自加
//history_close_Ind指最近的历史帧，是第几帧，由位姿的KD树求出
long laserCloudMap_Ind[100000];
long temp_laserCloudMap_Ind=0;
long history_close_Ind=0;

//我决定开始整活！这个是把每个地图帧的位姿保存在xyz里，之后用KD树找临近位姿，再根据位姿的ind从上面序号找临近帧的地图点进行匹配与闭环
pcl::PointCloud<PointType>::Ptr laserCloudMap_Pose(new pcl::PointCloud<PointType>());

//用于地图降采样
pcl::VoxelGrid<PointType> downSizeFilterMap;
pcl::VoxelGrid<PointType> downSizeFilterICP;

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

//把某点转换为地图点
void TransformToMapZero(PointType const *const pi, PointType *const po,PointType *const center)
{
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point;
    
    un_point= q_w_curr * point + t_w_curr;
 
    //输出一下
    po->x = un_point.x()-center->x;
    po->y = un_point.y()-center->y;
    po->z = un_point.z()-center->z;
    po->r = pi->r;
    po->g = pi->g;
    po->b = pi->b;
}

//当前帧所有点的保存句柄
void cloud_Callhandle(const sensor_msgs::PointCloud2Ptr &cloud)
{
    mBuf.lock();
    cloudBuf.push(cloud);
    mBuf.unlock();
}

//当前帧plane点的保存句柄
void cloud_plane_Callhandle(const sensor_msgs::PointCloud2Ptr &plane_cloud)
{
    mBuf.lock();
    planeBuf.push(plane_cloud);
    mBuf.unlock();
}
//当前帧odom运算出相对原点的位姿
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
	while(!odometryBuf.empty()&&!planeBuf.empty()&&!cloudBuf.empty())
	{
	    TicToc t_map;
	    
	    mBuf.lock();
	    
	    //记录时间戳
	    double timeLaserCloud = cloudBuf.front()->header.stamp.toSec();
	    double timeLaserCloudPlane = planeBuf.front()->header.stamp.toSec();
	    double timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
	    
	    if(timeLaserCloudPlane!=timeLaserOdometry||timeLaserCloud!=timeLaserOdometry)
	    {
		printf("unsync messeage! \n");
		mBuf.unlock();
		break;
	    }
	    
	    //清空上次面特征点云，并接收新的
	    laserCloudPlane->clear();
	    pcl::fromROSMsg(*planeBuf.front(), *laserCloudPlane);
	    planeBuf.pop();
	    
	    laserCloud->clear();
	    pcl::fromROSMsg(*cloudBuf.front(), *laserCloud);
	    cloudBuf.pop();
	    
	    downSizeFilterMap.setInputCloud(laserCloudPlane);
	    downSizeFilterMap.filter(*laserCloudPlane);
	    
	    downSizeFilterMap.setInputCloud(laserCloud);
	    downSizeFilterMap.filter(*laserCloud);
	    
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
		//推算当前地图下的四元数和位移
		q_w_curr = q_wmap_wodom * q_wodom_curr;
		t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
		
		pcl::PointCloud<PointType>::Ptr laserCloud_temp_Map(new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr laserCloud_temp_closure(new pcl::PointCloud<PointType>());
		//当已经处理了300帧以上时，对最近的200帧进行局部地图构建，进行帧-局部地图匹配，否则把所有点算入局部地图
		if(temp_laserCloudMap_Ind>300) 
		{
		    //把所有过去的pose送入KD树
		    pcl::KdTreeFLANN<PointType> kdtreePose;
		    kdtreePose.setInputCloud(laserCloudMap_Pose);
		    //当前位置
		    PointType pointpose;
		    pointpose.x=t_w_curr.x();
		    pointpose.y=t_w_curr.y();
		    pointpose.z=t_w_curr.z();
		    //KD树求解200个最近的pose，获取第几帧pointSearchInd，以及距当前帧的距离pointSearchSqDis
		    std::vector<int> pointSearchInd;std::vector<float> pointSearchSqDis;
		    kdtreePose.nearestKSearch(pointpose, 200, pointSearchInd, pointSearchSqDis);
		    //history_close_num为临近帧为历史帧（不在当前的前200帧的数量），hisory_close_flag是最近历史帧已有的标志，只有当前帧存在历史帧才会标志为true，不再进行判断
		    //history_close_Ind_temp为暂时保存的最近历史帧为第几帧，在满足历史帧在临近帧中有50个时，赋值给history_close_Ind
		    int history_close_num=0;bool hisory_close_flag=false;long history_close_Ind_temp=0;history_close_Ind=0;
		    for(int i=0;i<200;i++)
		    {
			//对于局部地图需要获取的plane点，要从有着全部plane点的laserCloudMap里取，从map_point_begin取到map_point_end
			//pointSearchInd[i]是第i近的帧是SLAM过程中的帧数，laserCloudMap_Ind[pointSearchInd[i]-1]是该帧起始的点的位置，
			//laserCloudMap_Ind[pointSearchInd[i]]是该帧结束的点的位置
			long map_point_begin=0;
			long map_point_end=0;
			if(pointSearchInd[i]!=0) map_point_begin = laserCloudMap_Ind[pointSearchInd[i]-1];
			map_point_end = laserCloudMap_Ind[pointSearchInd[i]];
			//从起始点录取到结束点
			for(long j=map_point_begin;j<map_point_end;j++)
			{
			    laserCloud_temp_Map->points.push_back(laserCloudMap->points[j]); 
			}
			//如果临近帧有历史帧，那么保存最近帧的位置，将最近帧已有标志打true，历史帧数量加1
			if(pointSearchInd[i]<temp_laserCloudMap_Ind-200&&pointSearchSqDis[i]<10) 
			{
			    if(!hisory_close_flag) history_close_Ind_temp=pointSearchInd[i];
			    hisory_close_flag = true;
			    history_close_num++;
			}
		    }
		    printf("history_close_num is %d\n",history_close_num);
		    //如果历史帧超过50，暂存变实际，后面会用history_close_Ind。
		    if(history_close_num>40) history_close_Ind = history_close_Ind_temp;
		    //降采样局部地图点
		    downSizeFilterMap.setInputCloud(laserCloud_temp_Map);
		    downSizeFilterMap.filter(*laserCloud_temp_Map);
		}
		//否则把所有点算入局部地图
		else
		{
		    downSizeFilterMap.setInputCloud(laserCloudMap);
		    downSizeFilterMap.filter(*laserCloud_temp_Map);
		}
	
		int plane_num = laserCloudPlane->points.size();
		
		for(int solve_num=0;solve_num<2;solve_num++)
		{
		    //把地图送入KD树
		    pcl::KdTreeFLANN<PointType> kdtreePlane;
		    kdtreePlane.setInputCloud(laserCloud_temp_Map);
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
			
			float dis_temp ;
			dis_temp = 2.0;
			//如果五个点里最远的那个也不超过2m，
			if (pointSearchSqDis[4] < dis_temp)
			{
			    //找五个点的中心点center，并计算五点形成平面的法向量norm
			    std::vector<Eigen::Vector3d> nearCorners;
			    Eigen::Vector3d center(0, 0, 0);
			    for (int j = 0; j < 5; j++)
			    {
				    Eigen::Vector3d tmp(laserCloud_temp_Map->points[pointSearchInd[j]].x,
							laserCloud_temp_Map->points[pointSearchInd[j]].y,
							laserCloud_temp_Map->points[pointSearchInd[j]].z);
				    center = center + tmp;
				    nearCorners.push_back(tmp);
			    }
			    center = center / 5.0;
			    Eigen::Matrix<double, 5, 3> matA0;
			    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
			    for (int j = 0; j < 5; j++)
			    {
				    matA0(j, 0) = laserCloud_temp_Map->points[pointSearchInd[j]].x;
				    matA0(j, 1) = laserCloud_temp_Map->points[pointSearchInd[j]].y;
				    matA0(j, 2) = laserCloud_temp_Map->points[pointSearchInd[j]].z;
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
				    Eigen::Vector3d vector_temp(laserCloud_temp_Map->points[pointSearchInd[j]].x-center.x(),
								laserCloud_temp_Map->points[pointSearchInd[j]].y-center.y(),
								laserCloud_temp_Map->points[pointSearchInd[j]].z-center.z());
			      
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
			options.max_num_iterations = 8;
			//进度是否发到STDOUT
			options.minimizer_progress_to_stdout = false;
			options.gradient_check_relative_precision = 1e-4;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
		    
			//优化后更迭地图和里程计之间的变换
			q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
			t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
		    }
		}
		
		//这里累积20帧点进行一次局部地图保存pcd
		static int local_map=0;
		local_map++;
		for(int i=0;i<int(laserCloud->points.size());i++)
		{
		    PointType pointseed;
		    TransformToMap(&laserCloud->points[i],&pointseed);
		    pointseed.r = 255;
		    pointseed.g = 255;
		    pointseed.b = 255;
		    laserCloud_local_map->points.push_back(pointseed);
		}
		if(local_map==20)
		{
		    local_map=0;
		    std::string str;static int pcd_num=0;
		    std::stringstream ss;
		    ss << pcd_num;
		    ss >> str;
		    pcd_num++;
		    std::string pcd_path = "/home/touchair/output/liom/";
		    downSizeFilterMap.setInputCloud(laserCloud_local_map);
		    downSizeFilterMap.filter(*laserCloud_local_map);
		    printf("local num is %d\n",int(laserCloud_local_map->points.size()));
		    std::string map_save_path=pcd_path+str+".pcd";
		    pcl::io::savePCDFileBinary(map_save_path, *laserCloud_local_map );
		    laserCloud_local_map->clear();
		}
		
		//如果最近历史帧的帧数（第几帧）history_close_Ind存在
		if(history_close_Ind)
		{
		    PointType center;
		    double sum_x=0;double sum_y=0;double sum_z=0;
		    for(int i=0;i<plane_num;i++)
		    {
			PointType pointseed;
			TransformToMap(&laserCloudPlane->points[i],&pointseed);
			sum_x=sum_x+pointseed.x;
			sum_y=sum_y+pointseed.y;
			sum_z=sum_z+pointseed.z;
		    }
		    center.x=sum_x/plane_num;
		    center.y=sum_y/plane_num;
		    center.z=sum_z/plane_num;
		    std::cout<<center.x<<" "<<center.y<<" "<<center.z<<std::endl;
		    
		    //对于本帧的所有点进行转换位姿，转换完累加到laserCloud_now_out
		    for(int i=0;i<int(laserCloud->points.size());i++)
		    {
			PointType pointseed;
			TransformToMapZero(&laserCloud->points[i],&pointseed,&center);
			pointseed.r = 255;
			pointseed.g = 255;
			pointseed.b = 255;
			laserCloud_now_out->points.push_back(pointseed);
		    }

		    //对于临近的7个局部地图进行读取
		    for(int i=-3;i<=3;i++)
		    {
				int pcd_read=history_close_Ind/20+i;
				if(pcd_read<0) continue;
				std::stringstream ss_read;std::string str_read;
				ss_read << pcd_read;
				ss_read >> str_read;
				std::string pcd_read_path = "/home/touchair/output/liom/";
				std::string map_read_path=pcd_read_path+str_read+".pcd";
				pcl::PointCloud<PointType>::Ptr laserCloud_local_read_map(new pcl::PointCloud<PointType>());
				
				//读了临近局部地图就累加在laserCloud_map_out里
				if (pcl::io::loadPCDFile<PointType>(map_read_path, *laserCloud_local_read_map )== -1)
				{
					PCL_ERROR("Couldn't read file local_map_pcd\n");
				}
				for(int j=0;j<int(laserCloud_local_read_map->size());j++)
				{
					PointType pointseed;
					pointseed.x=laserCloud_local_read_map->points[j].x-center.x;
					pointseed.y=laserCloud_local_read_map->points[j].y-center.y;
					pointseed.z=laserCloud_local_read_map->points[j].z-center.z;
					laserCloud_map_out->push_back(pointseed);
				}
		    }
		    
		    downSizeFilterICP.setInputCloud(laserCloud_now_out);
		    downSizeFilterICP.filter(*laserCloud_now_out);
		    downSizeFilterICP.setInputCloud(laserCloud_map_out);
		    downSizeFilterICP.filter(*laserCloud_map_out);
		    printf("plane num is %d\n",int(laserCloud_now_out->points.size()));
		    printf("map num is %d\n",int(laserCloud_map_out->points.size()));
		    
		    //构建icp
		    pcl::IterativeClosestPoint<PointType, PointType> icp;
		    pcl::PointCloud<PointType>::Ptr cloud_fina(new pcl::PointCloud<PointType>);
		    icp.setMaxCorrespondenceDistance(100);
		    icp.setMaximumIterations(100);
		    icp.setTransformationEpsilon(1e-8);
		    icp.setEuclideanFitnessEpsilon(1e-8);
		    icp.setRANSACIterations(0);
		    icp.setInputSource(laserCloud_now_out);
		    icp.setInputTarget(laserCloud_map_out);
		    //下面这句是必须的
		    icp.align(*cloud_fina);

		    //得分低于0.6可以进行闭环
 		    float icp_score = icp.getFitnessScore();
		    std::cout << "ICP converg flag:" << icp.hasConverged() << ". Fitness score: " << icp.getFitnessScore() << std::endl;
		    if(icp.hasConverged()&&icp_score<0.3)
  		    {
			//获取变换矩阵，求四元数q和位移t_vector
			Eigen::Matrix4f correctionLidarFrame;
			correctionLidarFrame = icp.getFinalTransformation();
			Eigen::Matrix3d r_matrix = Eigen::Matrix3d::Identity();
			for(int i=0;i<3;i++)
			{
			    for(int j=0;j<3;j++)
			    {
				r_matrix(i,j) = double(correctionLidarFrame(i,j));
			    }
			}
			Eigen::Vector3d t_icp(double(correctionLidarFrame(0,3)),double(correctionLidarFrame(1,3)),double(correctionLidarFrame(2,3)));
			Eigen::Quaterniond q_icp;
			q_icp = Eigen::Quaterniond ( r_matrix );
			
			//位姿变换
			std::cout<<"now1 t= "<<t_w_curr.x()<<" cen= "<<center.x<<std::endl;
			t_w_curr.x() = t_w_curr.x()-center.x;
			t_w_curr.y() = t_w_curr.y()-center.y;
			t_w_curr.z() = t_w_curr.z()-center.z;
			std::cout<<"now2 t= "<<t_w_curr.x()<<" cen= "<<center.x<<std::endl;
			t_w_curr = t_w_curr + q_w_curr * t_icp;
			q_w_curr = q_w_curr*q_icp ;
			std::cout<<"now3 t= "<<t_w_curr.x()<<" cen= "<<center.x<<std::endl;
			t_w_curr.x() = t_w_curr.x()+center.x;
			t_w_curr.y() = t_w_curr.y()+center.y;
			t_w_curr.z() = t_w_curr.z()+center.z;
			std::cout<<"now4 t= "<<t_w_curr.x()<<" cen= "<<center.x<<std::endl;
			q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
			t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
		    }	    
		    laserCloud_now_out->clear();
		    laserCloud_map_out->clear();
		}

		//保存当前plane点到laserCloudMap
		for(int i=0;i<plane_num;i++)
		{
		    PointType pointseed;
		    TransformToMap(&laserCloudPlane->points[i],&pointseed);
		    laserCloudMap->points.push_back(pointseed);
		}
	    }    

	    //记录当前帧保存后的点数便于之后查询，自加帧数
	    laserCloudMap_Ind[temp_laserCloudMap_Ind]=laserCloudMap->points.size();
	    temp_laserCloudMap_Ind++;
	    
	    //保存位姿，便于之后位姿KD树搜索
	    PointType pointpose;
	    pointpose.x=t_w_curr.x();
	    pointpose.y=t_w_curr.y();
	    pointpose.z=t_w_curr.z();
	    laserCloudMap_Pose->points.push_back(pointpose);
	    
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

	    if(temp_laserCloudMap_Ind%20==0&&temp_laserCloudMap_Ind!=0)
	    {
		//输出点云显示
		pcl::PointCloud<PointType>::Ptr laserCloudMap_Out(new pcl::PointCloud<PointType>());
		downSizeFilterMap.setInputCloud(laserCloudMap);
		downSizeFilterMap.filter(*laserCloudMap_Out);
		printf("laserCloudMap_Out num is %d",int(laserCloudMap_Out->points.size()));
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
    ros::init(argc, argv, "map");
    ros::NodeHandle n;
    
    downSizeFilterMap.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterICP.setLeafSize(0.2, 0.2, 0.2);
    
    ros::Subscriber LaserCloudmap = n.subscribe("/velodyne_cloud_map", 100, cloud_plane_Callhandle);
    ros::Subscriber LaserLocalOdom = n.subscribe("/laser_odom_to_init", 100, local_odom_Callhandle);
    ros::Subscriber cloud_sub = n.subscribe("/velodyne_cloud_all", 100, cloud_Callhandle);
    //输出map后的轨迹与位姿
    pubLaserPath = n.advertise<nav_msgs::Path>("/laser_map_path", 100);
    pubLaserMap  = n.advertise<sensor_msgs::PointCloud2>("/laser_map_cloud",100);
    
    boost::thread server(main_thread);
    
    ros::spin();
    return 0;
}