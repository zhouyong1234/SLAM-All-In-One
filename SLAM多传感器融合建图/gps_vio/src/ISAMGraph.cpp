// ISAMGraph.cpp
// Graph with ISAM solver
// Zhiang Chen, Aug 2020, zch@asu.edu

#include "gps_vio/ISAMGraph.h"

ISAMGraph::ISAMGraph()
{
}

void ISAMGraph::updateGPSVIO(nav_msgs::Odometry gps_odom, nav_msgs::Odometry vio_odom)
{

}

void ISAMGraph::updateGPS(nav_msgs::Odometry gps_odom)
{

}

void ISAMGraph::updateVIO(nav_msgs::Odometry vio_odom)
{

}

nav_msgs::Odometry ISAMGraph::getOdom()
{
	nav_msgs::Odometry currentOdom;
	return currentOdom;
}
