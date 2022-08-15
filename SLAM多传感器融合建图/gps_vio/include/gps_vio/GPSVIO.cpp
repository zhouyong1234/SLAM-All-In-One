// GPSVIO.tpp
// A Class that fuses GPS and VIO
// Zhiang Chen, Aug 2020, zch@asu.edu

template <class T> 
GPSVIO<T>::GPSVIO(const ros::NodeHandle& nh): nh_(nh)
{
	// start external calibration for tracking camera
	/*//online calibration
	sub_gps_ = nh_.subscribe("/gps/odom", 1, &GPSVIO::gpsCallback_, this, ros::TransportHints().tcpNoDelay());
	while(!calib_ && ros::ok())
	{
		ros::spinOnce();
		ROS_INFO("GPSVIO: Wait for tracking camera external calibration");
		ros::Duration(0.1).sleep();
	}*/

	// initialize ros interfaces
	gps_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/gps/odom", Queue_Size);
	vio_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/vio/odom", Queue_Size);
	gps_vio_sync_ = new message_filters::Synchronizer<GPS_VIO_POLICY>(GPS_VIO_POLICY(Queue_Size), *gps_sub_, *vio_sub_);
	gps_vio_sync_->registerCallback(boost::bind(&GPSVIO::gpsVioCallback_, this, _1, _2));
	
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gpsvio/odom", 1);
	//sub_gps_ = nh_.subscribe("/gps/odom", 1, &GPSVIO::gpsCallback, this, ros::TransportHints().tcpNoDelay());
	sub_vio_ = nh_.subscribe("/vio/odom", 1, &GPSVIO::vioCallback_, this, ros::TransportHints().tcpNoDelay());

	flag_gpsvio_ = false;
	// wait first gps+vio
	while(!flag_gpsvio_ && ros::ok())
	{
		ros::spinOnce();
		ROS_INFO("GPSVIO: Wait for gps+vio");
		ros::Duration(0.5).sleep();

	}
	/*// this is not needed; graph will be initialized in its constructor
	// initialize graph
	graph_->updateGPSVIO(newGPSVIO_GPS_, newGPSVIO_VIO_);
	// update process variables
	gpsVioVarUpdate_();
	*/
	// start timer to update graph
	timer_ = nh_.createTimer(ros::Duration(1.0/RATE), &GPSVIO::timerCallback_, this);  // RATE is defined in param.cpp
	ROS_INFO("GPSVIO has been initialized!");
}


template <class T> 
void GPSVIO<T>::gpsVioCallback_(const nav_msgs::Odometry::ConstPtr &gps_odom, const nav_msgs::Odometry::ConstPtr &vio_odom)
{
	newGPSVIO_GPS_ = *gps_odom;
	newGPSVIO_VIO_ = *vio_odom;
	flag_gpsvio_ = true;
}

template <class T>
void GPSVIO<T>::vioCallback_(const nav_msgs::Odometry::ConstPtr &vio_odom)
{
	newVIO_ = *vio_odom;
	flag_vio_ = true;
}

template <class T>
void GPSVIO<T>::timerCallback_(const ros::TimerEvent& event)
{
	if (flag_gpsvio_)  // highest priority
	{
		// update graph with node type gps_vio
		graph_->updateGPSVIO(newGPSVIO_GPS_, newGPSVIO_VIO_);
		// update process variables
		gpsVioVarUpdate_();
		// publish updated odometry
		publishOdom_();
	}
	else
	{
		if (flag_vio_)
		{
			// update graph with node type vio
			graph_->updateVIO(newVIO_);
			// update process variables
			vioVarUpdate_();
			// publish updated odometry
			publishOdom_();

		}
		else
		{
			odom_pub_.publish(current_odom_);
		}
	}

}

template <class T>
void GPSVIO<T>::gpsVioVarUpdate_()
{
	// all other variables will be updated because GPS+VIO has the highest priority
	//oldGPS_ = newGPSVIO_GPS_;
	oldVIO_ = newGPSVIO_VIO_;
	oldGPSVIO_GPS_ = newGPSVIO_GPS_;
	oldGPSGPS_VIO_ = newGPSVIO_VIO_;
	flag_gpsvio_ = false;
	//flag_gps_ = false;
	flag_vio_ = false;
	last_node_type_ = NODE_TYPE::GPS_VIO;
}



template <class T>
void GPSVIO<T>::vioVarUpdate_()
{
	oldVIO_ = newVIO_;
	flag_vio_ = false;
	last_node_type_ = NODE_TYPE::VIO;
}

template <class T>
void GPSVIO<T>::publishOdom_()
{
	current_odom_ = graph_->getOdom();
	odom_pub_.publish(current_odom_);
}

/*
template <class T>
void GPSVIO<T>::gpsVarUpdate_()
{
	oldGPS_ = newGPS_;
	flag_gps_ = false;
	last_node_type_ = NODE_TYPE::GPS;
}
*/

/*
template <class T>
void GPSVIO<T>::gpsCallback(const nav_msgs::Odometry::ConstPtr &gps_odom)
{
	newGPS_ = *gps_odom;
	flag_gps_ = true;
}
*/

/*//online calibration
template <class T>
void GPSVIO<T>::gpsCallback_(const nav_msgs::Odometry::ConstPtr &gps_odom)
{
	if (GPS_vec_.size() <= CALIB_NM )
	{
		GPS_vec_.push_back(*gps_odom);
	}
	else
	{
		std::vector<double> pos_Xs, pos_Ys, pos_Zs, orien_Xs, orien_Ys, orien_Zs, orien_Ws;
		int idx = GPS_vec_.size();
		while (--idx >= 0)
		{
			nav_msgs::Odometry gps_odom = GPS_vec_[idx];
			pos_Xs.push_back(gps_odom.pose.pose.position.x);
			pos_Ys.push_back(gps_odom.pose.pose.position.y);
			pos_Zs.push_back(gps_odom.pose.pose.position.z);
			orien_Xs.push_back(gps_odom.pose.pose.orientation.x);
			orien_Ys.push_back(gps_odom.pose.pose.orientation.y);
			orien_Zs.push_back(gps_odom.pose.pose.orientation.z);
			orien_Ws.push_back(gps_odom.pose.pose.orientation.w);
		}
		double c_pos_x = std::accumulate( pos_Xs.begin(), pos_Xs.end(), 0.0)/pos_Xs.size();
		double c_pos_y = std::accumulate( pos_Ys.begin(), pos_Ys.end(), 0.0)/pos_Ys.size();
		double c_pos_z = std::accumulate( pos_Zs.begin(), pos_Zs.end(), 0.0)/pos_Zs.size();
		double c_orien_x = std::accumulate( orien_Xs.begin(), orien_Xs.end(), 0.0)/orien_Xs.size();
		double c_orien_y = std::accumulate( orien_Ys.begin(), orien_Ys.end(), 0.0)/orien_Ys.size();
		double c_orien_z = std::accumulate( orien_Zs.begin(), orien_Zs.end(), 0.0)/orien_Zs.size();
		double c_orien_w = std::accumulate( orien_Ws.begin(), orien_Ws.end(), 0.0)/orien_Ws.size();
		Rot3 rot(c_orien_w, c_orien_x, c_orien_y, c_orien_z);
		Point3 point(c_pos_x, c_pos_y, c_pos_z);
		EMatrix_ =  Pose3(rot, point);
		calib_ = true;
		sub_gps_.shutdown();
		ROS_INFO("GPSVIO: camera external calibration done");
	}
}
*/

