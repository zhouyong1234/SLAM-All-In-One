// ISAM2Graph.h
// Graph with ISAM2 solver
// Zhiang Chen, Aug 2020, zch@asu.edu

#ifndef ISAM2GRAPH_H
#define ISAM2GRAPH_H

#include <nav_msgs/Odometry.h>

//#include <gtsam/geometry/Pose3.h>

#include <gps_vio/param.cpp>



class ISAM2Graph
{
public:
	ISAM2Graph();
	void updateGPSVIO(nav_msgs::Odometry gps_odom, nav_msgs::Odometry vio_odom);
	void updateGPS(nav_msgs::Odometry gps_odom);
	void updateVIO(nav_msgs::Odometry vio_odom);
	nav_msgs::Odometry getOdom();

private:

};



#endif 

