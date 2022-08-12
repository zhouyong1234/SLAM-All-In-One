#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <bits/stdc++.h>
#include <read_kaist_dataset/imu.h>
#include <read_kaist_dataset/encoder.h>
#include <read_kaist_dataset/gps.h>
#include <sensor_msgs/PointCloud.h>
#include "parameters.h"
#include <LocalCartesian.hpp>
#include <nav_msgs/Path.h>
using namespace std;

struct measurement{
	pair<double,sensor_msgs::PointCloudConstPtr> measurements_feature;
	vector<read_kaist_dataset::imu> measurements_imu;
	vector<read_kaist_dataset::encoder> measurements_encoder;
	vector<read_kaist_dataset::gps> measurements_gps;
};

struct state{
	Eigen::Vector3d p_w_i;
	Eigen::Vector3d v_w_i;
	Eigen::Matrix3d R_w_i;
	Eigen::Vector3d b_a;
	Eigen::Vector3d b_g;
	Eigen::Matrix<double, 15, 15> cov;
};

class Estimator{
public:
    Estimator();
	void processMeasurement(measurement& measure);
	//for initial
	bool initSystem();
	Eigen::Vector3d LonLatAlt2ENU(Eigen::Vector3d lon_lat_alt);
	Eigen::Matrix3d calcInitRotation(vector<read_kaist_dataset::imu>& imu_buf_initial);
	Eigen::Vector3d calcInitBiasAcc(vector<read_kaist_dataset::imu>& imu_buf_initial);
	Eigen::Matrix3d calcInitBiasAccCov(vector<read_kaist_dataset::imu>& imu_buf_initial);
	// for predict and update
	void estimateByMSCKF(measurement& measure);
	void predict(measurement& measure);
	void midPointIntegration(Eigen::Vector3d& acc, Eigen::Vector3d& gyro, double& dt);
	void update(measurement& measure);
	//for publish
	void state2nav_msgs();
	void gps2nav_msgs();

public:
    enum SolverFlag
    {
        INITIAL,
        MSCKF
    };

	SolverFlag solver_flag;
	vector<measurement> measure_buf_initial;
	Eigen::Vector3d lon_lat_alt_initial;
	Eigen::Matrix3d enu_cov_initial;
	state curr_state;
	state last_state;
	Eigen::Vector3d gravity;

	double last_feature_time;
	Eigen::Vector3d last_imu_acc;
	Eigen::Vector3d last_imu_gyro;

	nav_msgs::Path fused_path;
	nav_msgs::Path gps_path;
	Eigen::Vector3d curr_gps_path;
};

#endif