// TDGraph.cpp
// Graph based on Tridiagonal Matrix
// Zhiang Chen, Aug 2020, zch@asu.edu

#include "gps_vio/TDGraph.h"

TDGraph::TDGraph()
{
}

void TDGraph::updateGPSVIO(nav_msgs::Odometry gps_odom, nav_msgs::Odometry vio_odom)
{

}

void TDGraph::updateGPS(nav_msgs::Odometry gps_odom)
{

}

void TDGraph::updateVIO(nav_msgs::Odometry vio_odom)
{

}

nav_msgs::Odometry TDGraph::getOdom()
{
	nav_msgs::Odometry currentOdom;
	return currentOdom;
}
