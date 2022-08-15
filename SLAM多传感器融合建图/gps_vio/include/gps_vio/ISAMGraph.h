// ISAMGraph.h
// Graph with ISAM solver
// Zhiang Chen, Aug 2020, zch@asu.edu

#ifndef ISAMGRAPH_H
#define ISAMGRAPH_H

#include <nav_msgs/Odometry.h>
//#include <gtsam/geometry/Pose3.h>

#include <gps_vio/param.cpp>

//using namespace gtsam;

class ISAMGraph
{
public:
	ISAMGraph();
	void updateGPSVIO(nav_msgs::Odometry gps_odom, nav_msgs::Odometry vio_odom);
	void updateGPS(nav_msgs::Odometry gps_odom);
	void updateVIO(nav_msgs::Odometry vio_odom);
	nav_msgs::Odometry getOdom();

private:

};



#endif 

