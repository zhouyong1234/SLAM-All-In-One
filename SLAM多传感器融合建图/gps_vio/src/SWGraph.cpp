// SWGraph.cpp
// Sliding Window Graph with offline inference
// Zhiang Chen, Aug 2020, zch@asu.edu

#include "gps_vio/SWGraph.h"

SWGraph::SWGraph()
{
	Rot3 unit_rot = Rot3();
	Point3 zero_point(0, 0, 0);
	Pose3 init_pose(unit_rot, zero_point);
	gaussian_cov init_cov = noiseModel::Diagonal::Sigmas( (Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());
	swindow_.fixed_size_push(init_pose, init_pose, swindow_.vio_type, init_cov);
}

void SWGraph::updateGPSVIO(nav_msgs::Odometry gps_odom, nav_msgs::Odometry vio_odom)
{
	//ROS_INFO_STREAM(vio_odom);
	Pose3Gaussian vio_pose_gaussian = vio_odom_to_pose3_(vio_odom);
	//vio_pose_gaussian.cov->print("vio: ");
	Pose3Gaussian gps_pose_gaussian = gps_odom_to_pose3_(gps_odom);
	//gps_pose_gaussian.cov->print("gps: ");
	swindow_.fixed_size_push(vio_pose_gaussian.mean, vio_pose_gaussian.mean, swindow_.vio_type, vio_pose_gaussian.cov);
	swindow_.fixed_size_push(gps_pose_gaussian.mean, gps_pose_gaussian.mean, swindow_.gps_type, gps_pose_gaussian.cov);
	latest_odom_ = vio_odom;  // re-use stamp and frame_id
}

void SWGraph::updateVIO(nav_msgs::Odometry vio_odom)
{
	Pose3Gaussian vio_pose_gaussian = vio_odom_to_pose3_(vio_odom);
	swindow_.fixed_size_push(vio_pose_gaussian.mean, vio_pose_gaussian.mean, swindow_.vio_type, vio_pose_gaussian.cov);
	latest_odom_ = vio_odom; // re-use stamp and frame_id
}

nav_msgs::Odometry SWGraph::getOdom()
{
	solveGraph();
	return latest_odom_;
}

Pose3Gaussian SWGraph::vio_odom_to_pose3_(nav_msgs::Odometry odom)
{
	geometry_msgs::Quaternion quat = odom.pose.pose.orientation;
	geometry_msgs::Point p = odom.pose.pose.position;
	Rot3 rot(quat.w, quat.x, quat.y, quat.z);
	Point3 point(p.x, p.y, p.z);
	Pose3 pose(rot, point);

	vector<double> v{odom.pose.covariance[0], odom.pose.covariance[7], odom.pose.covariance[14],
  		  odom.pose.covariance[21], odom.pose.covariance[28], odom.pose.covariance[35]};
	bool zeros = std::all_of(v.begin(), v.end(), [](int i) { return i==0; });
	if (zeros || VIO_COV)
	{
		gaussian_cov cov = noiseModel::Diagonal::Variances(
						(Vector(6) << VIO_x, VIO_y, VIO_z, VIO_roll, VIO_pitch, VIO_yaw).finished());
		return Pose3Gaussian(pose, cov);
	}
	else
	{
		gaussian_cov cov = noiseModel::Diagonal::Variances(
				(Vector(6) << odom.pose.covariance[0], odom.pose.covariance[7], odom.pose.covariance[14],
						odom.pose.covariance[21], odom.pose.covariance[28], odom.pose.covariance[35]).finished());
		return Pose3Gaussian(pose, cov);
	}
}

Pose3Gaussian SWGraph::gps_odom_to_pose3_(nav_msgs::Odometry odom)
{
	geometry_msgs::Quaternion quat = odom.pose.pose.orientation;
	geometry_msgs::Point p = odom.pose.pose.position;
	Rot3 rot(quat.w, quat.x, quat.y, quat.z);
	Point3 point(p.x, p.y, p.z);
	Pose3 pose(rot, point);

	vector<double> v{odom.pose.covariance[0], odom.pose.covariance[7], odom.pose.covariance[14],
  		  odom.pose.covariance[21], odom.pose.covariance[28], odom.pose.covariance[35]};
	bool zeros = std::all_of(v.begin(), v.end(), [](int i) { return i==0; });
	if (zeros || GPS_COV)
	{
		gaussian_cov cov = noiseModel::Diagonal::Variances(
						(Vector(6) << GPS_x, GPS_y, GPS_z, GPS_roll, GPS_pitch, GPS_yaw).finished());
		return Pose3Gaussian(pose, cov);
	}
	else
	{
		gaussian_cov cov = noiseModel::Diagonal::Variances(
				(Vector(6) << odom.pose.covariance[0], odom.pose.covariance[7], odom.pose.covariance[14],
						odom.pose.covariance[21], odom.pose.covariance[28], odom.pose.covariance[35]).finished());
		return Pose3Gaussian(pose, cov);
	}

}


void SWGraph::pose3_to_odom_(Pose3 pose, vector<double> cov_vec, nav_msgs::Odometry& odom)
{
	Quaternion quat = pose.rotation().toQuaternion();
	odom.pose.pose.orientation.x = quat.x();
	odom.pose.pose.orientation.y = quat.y();
	odom.pose.pose.orientation.z = quat.z();
	odom.pose.pose.orientation.w = quat.w();
	odom.pose.pose.position.x = pose.x();
	odom.pose.pose.position.y = pose.y();
	odom.pose.pose.position.z = pose.z();
	copy(cov_vec.begin(), cov_vec.end(), odom.pose.covariance.begin() );
}

int SWGraph::solveGraph()
{
	if (swindow_.size <= 0)
		return 0;
	// reset graph and initial values
	graph_.erase(graph_.begin(), graph_.end());
	initials_.clear();
	// add the first node and value
	graph_.addPrior(1, swindow_.nodes[0], swindow_.covs[0]);
	initials_.insert(1, swindow_.values[0]);
	int last_type = swindow_.types[0];

	int last_key;
	for (int i=1; i<swindow_.size; i++)
	{
		int key = i+1;
		// add between factor
		if(swindow_.types[i] == swindow_.vio_type)
		{
			// add between factor with key-th node
			if(last_type == swindow_.vio_type)
			{
				Pose3 transform = swindow_.nodes[i-1].inverse() *swindow_.nodes[i];
				graph_.emplace_shared<BetweenFactor<Pose3> >(key-1, key, transform, swindow_.covs[i]);
				initials_.insert(key, swindow_.values[i]);
			}
			// add between factor with (key-1)-th node
			else
			{
				Pose3 transform = swindow_.nodes[i-2].inverse() *swindow_.nodes[i];
				graph_.emplace_shared<BetweenFactor<Pose3> >(key-2, key, transform, swindow_.covs[i]);
				initials_.insert(key, swindow_.values[i]);
			}
			last_type = swindow_.vio_type;
			last_key = key;
		}
		// add prior factor
		else
		{
			graph_.addPrior(key-1, swindow_.nodes[i], swindow_.covs[i]);
			last_type = swindow_.gps_type;
		}
	}
	//graph_.print();
	//initials_.print();
	Values results = LevenbergMarquardtOptimizer(graph_, initials_).optimize();
	Pose3 pose = results.at(last_key).cast<Pose3>();
	Marginals marginals(graph_, results);
	Matrix cov = marginals.marginalCovariance(last_key);
	vector<double> cov_vec(cov.data(), cov.data() + cov.rows() * cov.cols());
	pose3_to_odom_(pose, cov_vec, latest_odom_);
}
