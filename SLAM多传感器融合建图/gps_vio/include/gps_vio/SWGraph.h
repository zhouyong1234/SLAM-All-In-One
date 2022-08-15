// SWGraph.h
// Sliding Window Graph with offline inference
// Zhiang Chen, Aug 2020, zch@asu.edu

#ifndef SWGRAPH_H
#define SWGRAPH_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/Marginals.h>

#include <gps_vio/param.cpp>

using namespace gtsam;
using namespace std;

typedef noiseModel::Gaussian::shared_ptr gaussian_cov;

struct Pose3Gaussian{
	Pose3 mean;
	gaussian_cov cov;
	Pose3Gaussian(Pose3 m, gaussian_cov c): mean(m), cov(c){}
};

struct SWindow{
	SWindow(const int m): max(m){size=0; key_frontier=0;}
	const int max;
	deque<Pose3> nodes;
	deque<Pose3> values;
	const int vio_type = 0;
	const int gps_type = 1;
	deque<int> types;
	deque<gaussian_cov> covs;
	deque<int> keys;
	int size;
	int key_frontier;

	void fixed_size_push(Pose3 node, Pose3 value, int type, gaussian_cov cov)
	{
		key_frontier++;
		if (size++<(max+1))
		{
			nodes.push_back(node);
			values.push_back(value);
			types.push_back(type);
			covs.push_back(cov);
			keys.push_back(key_frontier);
		}
		if (size>max)
		{
			nodes.pop_front();
			values.pop_front();
			types.pop_front();
			covs.pop_front();
			keys.pop_front();
			size--;
			// if we pop a vio-gps node, we need to pop both vio and the following gps
			if (types[0] == gps_type)
			{
				nodes.pop_front();
				values.pop_front();
				types.pop_front();
				covs.pop_front();
				keys.pop_front();
				size--;
			}
		}
	}
};

class SWGraph
{
public:
	SWGraph();
	void updateGPSVIO(nav_msgs::Odometry gps_odom, nav_msgs::Odometry vio_odom);
	void updateVIO(nav_msgs::Odometry vio_odom);
	nav_msgs::Odometry getOdom();

private:
	NonlinearFactorGraph graph_;
	Values initials_;
	SWindow swindow_ = SWindow(MAX_NODE);
	nav_msgs::Odometry latest_odom_;

	Pose3Gaussian gps_odom_to_pose3_(nav_msgs::Odometry odom);
	Pose3Gaussian vio_odom_to_pose3_(nav_msgs::Odometry odom);
	void pose3_to_odom_(Pose3 pose, vector<double> cov_vec, nav_msgs::Odometry& odom);
	int solveGraph();
	int solveDynamicGraph();
};



#endif 

