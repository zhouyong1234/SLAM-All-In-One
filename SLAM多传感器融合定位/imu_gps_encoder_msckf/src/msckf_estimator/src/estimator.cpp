#include "estimator.h"

Estimator::Estimator(){
	solver_flag = INITIAL;
	measure_buf_initial.clear();
	lon_lat_alt_initial = Eigen::Vector3d::Zero();
	enu_cov_initial = Eigen::Matrix3d::Identity();
	gravity = Eigen::Vector3d::Zero();
	gravity(2,0) = -9.81007;
}

void Estimator::processMeasurement(measurement& measure){
	if(solver_flag == INITIAL){
		bool is_state = false;
		if(measure.measurements_encoder[0].leftEncoder == measure.measurements_encoder.back().leftEncoder &&
			measure.measurements_encoder[0].rightEncoder == measure.measurements_encoder.back().rightEncoder)
			is_state = true;
		if(is_state)
			measure_buf_initial.push_back(measure);
		else
			measure_buf_initial.clear();
		if(measure_buf_initial.size() == 10){
			ROS_INFO("measures have enough for system initial!");
			if(initSystem()){
				measure_buf_initial.clear();
				solver_flag = MSCKF;
				ROS_INFO("init system success!");
			}
		}
	}else{
		estimateByMSCKF(measure);
	}
}

bool Estimator::initSystem(){
	// position
	bool init_gps = false;
	double last_gps_time = 0;
	vector<read_kaist_dataset::gps> gps_buf_initial;
	for(auto& m:measure_buf_initial){
		if(!init_gps){
			init_gps = true;
			last_gps_time = m.measurements_gps[0].timeStamp;
			gps_buf_initial.push_back(m.measurements_gps[0]);
		}
		for(auto& gps:m.measurements_gps){
			if(gps.timeStamp == last_gps_time)
				continue;
			last_gps_time = gps.timeStamp;
			gps_buf_initial.push_back(gps);
		}
	}
	double cov_x_inverse = 0, cov_y_inverse = 0, cov_z_inverse = 0;
	for(auto& gps:gps_buf_initial){
		lon_lat_alt_initial.x() += gps.longitude;
		lon_lat_alt_initial.y() += gps.latitude;
		lon_lat_alt_initial.z() += gps.altitude;
		cov_x_inverse += 1.0/gps.cov00;
		cov_y_inverse += 1.0/gps.cov11;
		cov_z_inverse += 1.0/gps.cov22;
	}
	lon_lat_alt_initial.x() /= gps_buf_initial.size();
	lon_lat_alt_initial.y() /= gps_buf_initial.size();
	lon_lat_alt_initial.z() /= gps_buf_initial.size();
	double cov_x = 1.0/cov_x_inverse, cov_y = 1.0/cov_y_inverse, cov_z = 1.0/cov_z_inverse;
	enu_cov_initial(0,0) = cov_x;
	enu_cov_initial(1,1) = cov_y;
	enu_cov_initial(2,2) = cov_z;

	//velocity
	curr_state.v_w_i = Eigen::Vector3d::Zero();
	curr_state.cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();

	//rotation
	bool init_imu = false;
	double last_imu_time = 0.0;
	vector<read_kaist_dataset::imu> imu_buf_initial; 
	for(auto& m:measure_buf_initial){
		if(!init_imu){
			init_imu = true;
			last_imu_time = m.measurements_imu[0].timeStamp;
			imu_buf_initial.push_back(m.measurements_imu[0]);
		}
		for(auto& imu:m.measurements_imu){
			if(imu.timeStamp == last_imu_time)
				continue;
			last_imu_time = imu.timeStamp;
			imu_buf_initial.push_back(imu);
		}
	}
	curr_state.R_w_i = calcInitRotation(imu_buf_initial);
	curr_state.cov.block<3, 3>(6, 6) = pow(10.0 * M_PI / 180.0 , 2.0) * Eigen::Matrix3d::Identity();
	curr_state.cov(8, 8) = pow(100 * M_PI / 180.0, 2.0);

	curr_state.p_w_i = Eigen::Vector3d::Zero() + curr_state.R_w_i * ( t_o_i - t_o_g);
	curr_state.cov.block<3, 3>(0, 0) = enu_cov_initial;

	//bias acc
	curr_state.b_a = Estimator::calcInitBiasAcc(imu_buf_initial);
	curr_state.cov.block<3, 3>(9, 9) = calcInitBiasAccCov(imu_buf_initial);

	//bias gyro
	Eigen::Vector3d sum_gyro(0.0, 0.0, 0.0);
    for (const auto imu_data : imu_buf_initial) {
		Eigen::Vector3d gyro_vec(imu_data.gx, imu_data.gy, imu_data.gz);
		sum_gyro = sum_gyro + gyro_vec;
	}
	sum_gyro /= (double)imu_buf_initial.size();
	curr_state.b_g = sum_gyro;
	double dx = 0, dy = 0, dz = 0;
    for (const auto imu_data : imu_buf_initial) {
		Eigen::Vector3d gyro_vec(imu_data.gx, imu_data.gy, imu_data.gz);
		Eigen::Vector3d delta_gyro = curr_state.b_g - gyro_vec;
		dx += pow(delta_gyro(0,0), 2.0);
		dy += pow(delta_gyro(1,0), 2.0);
		dz += pow(delta_gyro(2,0), 2.0);
	}
	dx /= (double)imu_buf_initial.size();
	dy /= (double)imu_buf_initial.size();
	dz /= (double)imu_buf_initial.size();
	curr_state.cov(12, 12) = dx;
	curr_state.cov(13, 13) = dy;
	curr_state.cov(14, 14) = dz;

	measurement last_measure = measure_buf_initial.back();
	double feature_time = last_measure.measurements_feature.first;
	double back_time = last_measure.measurements_imu[last_measure.measurements_imu.size()-2].timeStamp;
	double front_time = last_measure.measurements_imu.back().timeStamp;
	Eigen::Vector3d back_acc = Eigen::Vector3d::Zero();
	back_acc(0,0) = last_measure.measurements_imu[last_measure.measurements_imu.size()-2].ax;
	back_acc(1,0) = last_measure.measurements_imu[last_measure.measurements_imu.size()-2].ay;
	back_acc(2,0) = last_measure.measurements_imu[last_measure.measurements_imu.size()-2].az;
	Eigen::Vector3d front_acc = Eigen::Vector3d::Zero();
	front_acc(0,0) = last_measure.measurements_imu.back().ax;
	front_acc(1,0) = last_measure.measurements_imu.back().ay;
	front_acc(2,0) = last_measure.measurements_imu.back().az;
	last_imu_acc = back_acc + (feature_time - back_time) / (front_time - back_time) * (front_acc - back_acc);
	Eigen::Vector3d back_gyro = Eigen::Vector3d::Zero();
	back_gyro(0,0) = last_measure.measurements_imu[last_measure.measurements_imu.size()-2].gx;
	back_gyro(1,0) = last_measure.measurements_imu[last_measure.measurements_imu.size()-2].gy;
	back_gyro(2,0) = last_measure.measurements_imu[last_measure.measurements_imu.size()-2].gz;
	Eigen::Vector3d front_gyro = Eigen::Vector3d::Zero();
	front_gyro(0,0) = last_measure.measurements_imu.back().gx;
	front_gyro(1,0) = last_measure.measurements_imu.back().gy;
	front_gyro(2,0) = last_measure.measurements_imu.back().gz;
	last_imu_gyro = back_gyro + (feature_time - back_time) / (front_time - back_time) * (front_gyro - back_gyro);

	last_feature_time = feature_time;
	last_state = curr_state;

	return true;
}

Eigen::Vector3d Estimator::LonLatAlt2ENU(Eigen::Vector3d lon_lat_alt){
	static GeographicLib::LocalCartesian local_cartesian;
	local_cartesian.Reset(lon_lat_alt_initial(1), lon_lat_alt_initial(0), lon_lat_alt_initial(2));
	Eigen::Vector3d enu_pose;
    local_cartesian.Forward( lon_lat_alt(1), lon_lat_alt(0), lon_lat_alt(2),
		enu_pose.data()[0], enu_pose.data()[1], enu_pose.data()[2]);
	return enu_pose;
}

Eigen::Matrix3d Estimator::calcInitRotation(vector<read_kaist_dataset::imu>& imu_buf_initial){
	Eigen::Vector3d sum_acc(0.0, 0.0, 0.0);
    for (const auto imu_data : imu_buf_initial) {
		Eigen::Vector3d acc_vec(imu_data.ax, imu_data.ay, imu_data.az);
		sum_acc = sum_acc + acc_vec;
	}
	const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buf_initial.size();

	const Eigen::Vector3d& z_axis = mean_acc.normalized(); 
	Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d R_iw;
    R_iw.block<3, 1>(0, 0) = x_axis;
    R_iw.block<3, 1>(0, 1) = y_axis;
    R_iw.block<3, 1>(0, 2) = z_axis;

	return R_iw.inverse();
}

Eigen::Vector3d Estimator::calcInitBiasAcc(vector<read_kaist_dataset::imu>& imu_buf_initial){
	Eigen::Vector3d sum_acc(0.0, 0.0, 0.0);
    for (const auto imu_data : imu_buf_initial) {
		Eigen::Vector3d acc_vec(imu_data.ax, imu_data.ay, imu_data.az);
		sum_acc = sum_acc + acc_vec;
	}
	const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buf_initial.size();

	Eigen::Vector3d bias_acc = Eigen::Vector3d::Zero();
	bias_acc = mean_acc + curr_state.R_w_i.inverse() * gravity;
	return bias_acc;
}

Eigen::Matrix3d Estimator::calcInitBiasAccCov(vector<read_kaist_dataset::imu>& imu_buf_initial){
	Eigen::Matrix3d bias_acc_cov_initial = Eigen::Matrix3d::Zero();
	double dx = 0, dy = 0, dz = 0;
    for (const auto imu_data : imu_buf_initial) {
		Eigen::Vector3d acc_vec(imu_data.ax, imu_data.ay, imu_data.az);
		Eigen::Vector3d bias_acc = acc_vec + curr_state.R_w_i.inverse() * gravity;
		Eigen::Vector3d delta_bias_acc = curr_state.b_a - bias_acc;
		dx += pow(delta_bias_acc(0,0), 2.0);
		dy += pow(delta_bias_acc(1,0), 2.0);
		dz += pow(delta_bias_acc(2,0), 2.0);
	}
	dx /= (double)imu_buf_initial.size();
	dy /= (double)imu_buf_initial.size();
	dz /= (double)imu_buf_initial.size();
	bias_acc_cov_initial(0, 0) = dx;
	bias_acc_cov_initial(1, 1) = dy;
	bias_acc_cov_initial(2, 2) = dz;

	return bias_acc_cov_initial;
}

void Estimator::estimateByMSCKF(measurement& measure){
	predict(measure);
	update(measure);
}

void Estimator::predict(measurement& measure){
	double feature_time = measure.measurements_feature.first;
	double last_imu_time = last_feature_time;
	for(auto& imu:measure.measurements_imu){
		if(imu.timeStamp < feature_time){
			Eigen::Vector3d curr_imu_acc(imu.ax, imu.ay, imu.az);
			Eigen::Vector3d curr_imu_gyro(imu.gx, imu.gy, imu.gz);
			Eigen::Vector3d mid_imu_acc = 0.5 * (last_imu_acc + curr_imu_acc) - curr_state.b_a;
			Eigen::Vector3d mid_imu_gyro = 0.5 * (last_imu_gyro + curr_imu_gyro) - curr_state.b_g;
			double dt = imu.timeStamp - last_imu_time;
			midPointIntegration(mid_imu_acc, mid_imu_gyro, dt);
			last_imu_acc = curr_imu_acc;
			last_imu_gyro = curr_imu_gyro;
			last_imu_time = imu.timeStamp;
		}else{
			double curr_imu_time = imu.timeStamp;
			Eigen::Vector3d curr_imu_acc(imu.ax, imu.ay, imu.az);
			curr_imu_acc = last_imu_acc + (feature_time - last_imu_time) / (curr_imu_time - last_imu_time) * (curr_imu_acc - last_imu_acc);
			Eigen::Vector3d mid_imu_acc = 0.5 * (last_imu_acc + curr_imu_acc) - curr_state.b_a;
			Eigen::Vector3d curr_imu_gyro(imu.gx, imu.gy, imu.gz);
			curr_imu_gyro = last_imu_gyro + (feature_time - last_imu_time) / (curr_imu_time - last_imu_time) * (curr_imu_gyro - last_imu_gyro);
			Eigen::Vector3d mid_imu_gyro = 0.5 * (last_imu_gyro + curr_imu_gyro) - curr_state.b_g;
			double dt = feature_time - last_imu_time;
			midPointIntegration(mid_imu_acc, mid_imu_gyro, dt);
			last_imu_acc = curr_imu_acc;
			last_imu_gyro = curr_imu_gyro;
			last_imu_time = feature_time;
		}
	}
	last_feature_time = feature_time;
	return;
}

void Estimator::midPointIntegration(Eigen::Vector3d& acc, Eigen::Vector3d& gyro, double& dt){
	//position
	curr_state.p_w_i = curr_state.p_w_i + curr_state.v_w_i * dt + 0.5 * (curr_state.R_w_i * acc + gravity) * dt * dt;
	//velocity
	curr_state.v_w_i = curr_state.v_w_i + (curr_state.R_w_i * acc + gravity) * dt;
	//rotation
	Eigen::Vector3d delta_angle_axis = dt  * gyro;
    if (delta_angle_axis.norm() > 1e-12)
        curr_state.R_w_i = curr_state.R_w_i * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
	//covariance
    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(3, 6)   = - curr_state.R_w_i * GetSkewMatrix(acc) * dt;
    Fx.block<3, 3>(3, 9)   = - curr_state.R_w_i * dt;
    if (delta_angle_axis.norm() > 1e-12) 
        Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix().transpose();
    else 
        Fx.block<3, 3>(6, 6).setIdentity();
    Fx.block<3, 3>(6, 12)  = - Eigen::Matrix3d::Identity() * dt;

    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = dt * dt * acc_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = dt * dt * gyro_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = dt * acc_bias_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = dt * gyro_bias_noise * Eigen::Matrix3d::Identity();

	curr_state.cov = Fx * curr_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();

	return;
}

void Estimator::update(measurement& measure){
	//update by gps
	double feature_time = measure.measurements_feature.first;
	double back_gps_time = measure.measurements_gps[0].timeStamp;
	double front_gps_time = measure.measurements_gps.back().timeStamp;
	Eigen::Vector3d back_lon_lat_alt(measure.measurements_gps[0].longitude,
		measure.measurements_gps[0].latitude, 
		measure.measurements_gps[0].altitude);
	Eigen::Vector3d front_lon_lat_alt(measure.measurements_gps.back().longitude,
		measure.measurements_gps.back().latitude, 
		measure.measurements_gps.back().altitude);
	Eigen::Vector3d mid_lon_lat_alt = back_lon_lat_alt + 
		(feature_time - back_gps_time) / (front_gps_time - back_gps_time) * (front_lon_lat_alt - back_lon_lat_alt);
	Eigen::Matrix3d mid_gps_cov = Eigen::Matrix3d::Identity();
	mid_gps_cov(0,0) = max(measure.measurements_gps[0].cov00, measure.measurements_gps.back().cov00);
	mid_gps_cov(1,1) = max(measure.measurements_gps[0].cov11, measure.measurements_gps.back().cov11);
	mid_gps_cov(2,2) = max(measure.measurements_gps[0].cov22, measure.measurements_gps.back().cov22);

	//update by wheel encoder


	//calculate gps residual and jacobian
	Eigen::Vector3d mid_enu_pose = LonLatAlt2ENU(mid_lon_lat_alt);
	curr_gps_path = mid_enu_pose;
	Eigen::Vector3d residual_gps = mid_enu_pose - (curr_state.p_w_i - curr_state.R_w_i * ( t_o_i - t_o_g));
	Eigen::Matrix<double, 3, 15> jacobian_gps;
	jacobian_gps.setZero();
	jacobian_gps.block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
    jacobian_gps.block<3, 3>(0, 6)  = -1.0 * curr_state.R_w_i *GetSkewMatrix( t_o_i - t_o_g);

	//update by gps
	const Eigen::MatrixXd K = curr_state.cov * jacobian_gps.transpose() * 
		(jacobian_gps * curr_state.cov * jacobian_gps.transpose() + mid_gps_cov).inverse();
	Eigen::VectorXd delta_x = K * residual_gps;
	curr_state.p_w_i += delta_x.block<3, 1>(0, 0);
	curr_state.v_w_i += delta_x.block<3, 1>(3, 0);
    if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) 
		curr_state.R_w_i *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
    curr_state.b_a  += delta_x.block<3, 1>(9, 0);
	curr_state.b_g += delta_x.block<3, 1>(12, 0);
    const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * jacobian_gps;
    curr_state.cov = I_KH * curr_state.cov  * I_KH.transpose() + K * mid_gps_cov * K.transpose();

	return;
}

void Estimator::state2nav_msgs(){
    fused_path.header.frame_id = "world";
    fused_path.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = fused_path.header;

    pose.pose.position.x = curr_state.p_w_i(0,0);
    pose.pose.position.y = curr_state.p_w_i(1,0);
    pose.pose.position.z = curr_state.p_w_i(2,0);

    const Eigen::Quaterniond Q_w_i(curr_state.R_w_i);
    pose.pose.orientation.x = Q_w_i.x();
    pose.pose.orientation.y = Q_w_i.y();
    pose.pose.orientation.z = Q_w_i.z();
    pose.pose.orientation.w = Q_w_i.w();

    fused_path.poses.push_back(pose);
	return ;
}

void Estimator::gps2nav_msgs(){
    gps_path.header.frame_id = "world";
    gps_path.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = gps_path.header;

    pose.pose.position.x = curr_gps_path(0,0);
    pose.pose.position.y = curr_gps_path(1,0);
    pose.pose.position.z = curr_gps_path(2,0);

    const Eigen::Quaterniond Q_w_i(curr_state.R_w_i);
    pose.pose.orientation.x = Q_w_i.x();
    pose.pose.orientation.y = Q_w_i.y();
    pose.pose.orientation.z = Q_w_i.z();
    pose.pose.orientation.w = Q_w_i.w();

    gps_path.poses.push_back(pose);
	return ;
}