// TDGraph.h
// Graph based on Tridiagonal Matrix
// Zhiang Chen, Aug 2020, zch@asu.edu

#ifndef TDGRAPH_H
#define TDGRAPH_H

#include <nav_msgs/Odometry.h>

//#include <gtsam/geometry/Pose3.h>

#include <gps_vio/param.cpp>



class TDGraph
{
public:
	TDGraph();
	void updateGPSVIO(nav_msgs::Odometry gps_odom, nav_msgs::Odometry vio_odom);
	void updateGPS(nav_msgs::Odometry gps_odom);
	void updateVIO(nav_msgs::Odometry vio_odom);
	nav_msgs::Odometry getOdom();

private:

};



#endif 

