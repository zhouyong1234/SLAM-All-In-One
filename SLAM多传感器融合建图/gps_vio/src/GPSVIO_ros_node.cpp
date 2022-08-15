// GPSVIO_ros_node.cpp
// A ros node for gps and vio fusion
// Zhiang Chen, Aug 2020, zch@asu.edu

#include <ros/ros.h>
#include "gps_vio/GPSVIO.h"

#include "gps_vio/SWGraph.h"
#include "gps_vio/ISAMGraph.h"
#include "gps_vio/ISAM2Graph.h"
#include "gps_vio/TDGraph.h"



int main(int argc, char** argv) 
{
	ros::init(argc, argv, "gps_vio");
	ros::NodeHandle nh;
	
	GPSVIO<SWGraph> gps_vio(nh);
	// parameters are stored in "include/gps_vio/param.cpp"

	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();
	return 0;
}
