// GPSVIO.h
// A Class that fuses GPS and VIO
// Zhiang Chen, Aug 2020, zch@asu.edu

#ifndef GPSVIO_H
#define GPSVIO_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "gps_vio/SWGraph.h"
#include "gps_vio/ISAMGraph.h"
#include "gps_vio/ISAM2Graph.h"
#include "gps_vio/TDGraph.h"

#include "gps_vio/param.cpp"

enum class NODE_TYPE {
  GPS_VIO,
  VIO,
};

template <class T>
class GPSVIO
{
public:
	GPSVIO(const ros::NodeHandle& nh);
	
protected:
	/****************** ros ******************/
	ros::NodeHandle nh_;
	ros::Publisher odom_pub_;
	ros::Subscriber sub_vio_;
	void vioCallback_(const nav_msgs::Odometry::ConstPtr &vio_odom);

	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> GPS_VIO_POLICY;
	message_filters::Subscriber<nav_msgs::Odometry> *gps_sub_; // message filter
	message_filters::Subscriber<nav_msgs::Odometry> *vio_sub_;
	message_filters::Synchronizer<GPS_VIO_POLICY> *gps_vio_sync_;
	void gpsVioCallback_(const nav_msgs::Odometry::ConstPtr &gps_odom, const nav_msgs::Odometry::ConstPtr &vio_odom);


	ros::Timer timer_;
	void timerCallback_(const ros::TimerEvent& event);

	/****************** graph ******************/
	T* graph_ = new T();

	/****************** process ******************/
	//nav_msgs::Odometry newGPS_;
	//nav_msgs::Odometry oldGPS_;
	nav_msgs::Odometry newVIO_;
	nav_msgs::Odometry oldVIO_;
	nav_msgs::Odometry newGPSVIO_GPS_;
	nav_msgs::Odometry oldGPSVIO_GPS_;
	nav_msgs::Odometry newGPSVIO_VIO_;
	nav_msgs::Odometry oldGPSGPS_VIO_;
	nav_msgs::Odometry current_odom_;
	//bool flag_gps_ = false;
	bool flag_vio_ = false;
	bool flag_gpsvio_ = false;
	NODE_TYPE last_node_type_;

	void gpsVioVarUpdate_();
	//void gpsVarUpdate_();
	void vioVarUpdate_();
	void publishOdom_();
	/****************** calibration ******************/
	/*ros::Subscriber sub_gps_;
	void gpsCallback_(const nav_msgs::Odometry::ConstPtr &gps_odom);
	bool calib_ = false;
	std::vector<nav_msgs::Odometry> GPS_vec_;
	Pose3 EMatrix_;
	*/

};

#include "gps_vio/GPSVIO.cpp"

#endif 

