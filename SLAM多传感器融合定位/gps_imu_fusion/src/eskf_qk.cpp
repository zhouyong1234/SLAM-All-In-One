//
// Created by meng on 2021/2/19.
//
#include "eskf_qk.h"
#include "../3rd/sophus/se3.hpp"

ESKFQK::ESKFQK(const YAML::Node &node) {

    // 判断使用哪一组数据
    std::string  data_path = node["data_path"].as<std::string>();
    std::string cov_node_string;
    if(data_path == "/data/raw_data") {
        cov_node_string = "covariance";
    } else if(data_path == "/data/raw_data1"){
        cov_node_string = "covariance1";
    } else {
        printf("no corres covariance");
        exit(0);
    }

    double gravity = node["earth"]["gravity"].as<double>();
    double cov_prior_posi = node["ESKFQK"][cov_node_string]["prior"]["posi"].as<double>();
    double cov_prior_vel = node["ESKFQK"][cov_node_string]["prior"]["vel"].as<double>();
    double cov_prior_ori = node["ESKFQK"][cov_node_string]["prior"]["ori"].as<double>();
    double cov_prior_accel = node["ESKFQK"][cov_node_string]["prior"]["accel_delta"].as<double>();
    double cov_prior_gyro = node["ESKFQK"][cov_node_string]["prior"]["gyro_delta"].as<double>();
    double cov_prior_g = node["ESKFQK"][cov_node_string]["prior"]["g_delta"].as<double>();
    double cov_process_vel = node["ESKFQK"][cov_node_string]["process"]["vel_delta"].as<double>();
    double cov_process_ori = node["ESKFQK"][cov_node_string]["process"]["ori_delta"].as<double>();
    double cov_process_accel = node["ESKFQK"][cov_node_string]["process"]["accel_delta"].as<double>();
    double cov_process_gyro = node["ESKFQK"][cov_node_string]["process"]["gyro_delta"].as<double>();
    double cov_measurement_posi = node["ESKFQK"][cov_node_string]["measurement"]["posi"].as<double>();
    g_ = Eigen::Vector3d(0.0, 0.0, -gravity);
    
    X_.setZero(); // 初始化为零矩阵
    Y_.setZero(); // 初始化为零矩阵

    Fi_.block<12,12>(3,0) = Eigen::Matrix<double,12,12>::Identity();
    SetCovarianceQi(cov_process_vel, cov_process_ori,cov_process_accel,cov_process_gyro);
    SetCovarianceP(cov_prior_posi, cov_prior_vel, cov_prior_ori,
                   cov_prior_accel, cov_prior_gyro, cov_prior_g);
    

    SetCovarianceV(cov_measurement_posi);
}

void ESKFQK::SetCovarianceV(double measurement_noise) {
    V_ = Eigen::Matrix3d::Identity()*measurement_noise;
}

void ESKFQK::SetCovarianceQi(double v_noise, double q_noise, double a_noise, double w_noise) {
    Qi_.setZero();
    Qi_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * v_noise * v_noise; // 平方
    Qi_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * q_noise * q_noise;
    Qi_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * a_noise * a_noise;
    Qi_.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * w_noise * w_noise;
}


// 设置P矩阵
void ESKFQK::SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
                          double accel_noise, double gyro_noise, double g_noise) {
    P_.setZero();
    P_.block<3,3>(INDEX_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * posi_noise;
    P_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity() * velo_noise;
    P_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix3d::Identity() * ori_noise;
    P_.block<3,3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) = Eigen::Matrix3d::Identity() * accel_noise;
    P_.block<3,3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) = Eigen::Matrix3d::Identity() * gyro_noise;
    P_.block<3,3>(INDEX_STATE_G, INDEX_STATE_G) = Eigen::Matrix3d::Identity() * g_noise;
}

bool ESKFQK::Init(const GPSData &curr_gps_data, const IMUData &curr_imu_data) {
    init_pose_ = curr_gps_data.position_ned;
    pose_ = init_pose_;

    init_velocity_ = curr_gps_data.true_velocity; // 用真实速度初始化
    velocity_ = init_velocity_;
    // 前右地
    Eigen::Quaterniond Q = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());
    init_q_ = Q;
    q_ = init_q_;
    imu_data_buff_.clear(); // 这时ESKF中的imu数据
    imu_data_buff_.push_back(curr_imu_data);

    curr_gps_data_ = curr_gps_data;
    return true;
}

// void ESKFQK::GetFGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y) {
//     F = Ft_;
//     G = G_;
//     Y = Y_;
// }

bool ESKFQK::ObservationOfErrorState()
{
    Hx_.setZero();
    Hx_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    // X_dx_;
    X_dx_.block<6,6>(0,0) = Eigen::Matrix<double,6,6>::Identity();
    X_dx_.block<9,9>(10,9) = Eigen::Matrix<double,9,9>::Identity();
    double qx = q_.x();
    double qy = q_.y();
    double qz = q_.z();
    double qw = q_.w();
    X_dx_.block<4,3>(6,6) = 0.5*(Eigen::Matrix<double,4,3>()<<-qx,-qy,-qz,
                                                                qw,-qz,qy,
                                                                qz,qw,-qx,
                                                                -qy,qx,qw).finished();
    H_ = Hx_*X_dx_;

    K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() +V_).inverse(); // kalman增益
    X_ = K_*(curr_gps_data_.position_ned - pose_);

    P_ = (TypeMatrixP::Identity() - K_ * H_) * P_*(TypeMatrixP::Identity() - K_ * H_).transpose()+K_*V_*K_.transpose();
    return true;
}

bool ESKFQK::Correct(const GPSData &curr_gps_data) {
    curr_gps_data_ = curr_gps_data;
    ObservationOfErrorState();
    EliminateError();
    ResetState();

    return true;
}

// IMU数据预测
bool ESKFQK::Predict(const IMUData &curr_imu_data) {
    imu_data_buff_.push_back(curr_imu_data);

    IMUData last_imu_data = imu_data_buff_.at(0);

    double delta_t = curr_imu_data.time - last_imu_data.time;
    Eigen::Vector3d a_m = curr_imu_data.linear_accel;
    Eigen::Vector3d w_m = curr_imu_data.angle_velocity;

    UpdateOdomEstimation(a_m,w_m,delta_t); // 更新 角度  速度 位置  PVQ
    UpdateErrorState(a_m,w_m,delta_t); // 更新误差状态，只与R和accel相关

    imu_data_buff_.pop_front();
    return true;
}

bool ESKFQK::UpdateErrorState(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, const double dt) {
    Eigen::Matrix3d A_M = BuildSkewMatrix(a_m-accel_bias_);

    // Fx
    Fx_.setZero(); // 初始化为零矩阵
    Fx_.block<3,3>(0,0) = Fx_.block<3,3>(3,3) = Fx_.block<3,3>(9,9) = Fx_.block<3,3>(12,12) = Fx_.block<3,3>(15,15)= Eigen::Matrix3d::Identity();
  
    Eigen::Matrix3d R = q_.toRotationMatrix();
    Fx_.block<3,3>(INDEX_STATE_POSI,INDEX_STATE_VEL) =  Eigen::Matrix3d::Identity()*dt;
    Fx_.block<3,3>(INDEX_STATE_VEL,INDEX_STATE_ORI) =  -R*A_M*dt;
    Fx_.block<3,3>(INDEX_STATE_VEL,INDEX_STATE_ACC_BIAS) =  -R*dt;
    Fx_.block<3,3>(INDEX_STATE_VEL,INDEX_STATE_GYRO_BIAS) = Eigen::Matrix3d::Identity()*dt;
    

    Eigen::AngleAxisd  AngleAxis_w(((w_m-gyro_bias_)*dt).norm(),((w_m-gyro_bias_)*dt).normalized());
    Eigen::Matrix3d R_w(AngleAxis_w);
    Fx_.block<3,3>(INDEX_STATE_ORI,INDEX_STATE_ORI) = R_w.transpose();//公式157c
    // Fx_.block<3,3>(INDEX_STATE_ORI,INDEX_STATE_ORI) = Eigen::Matrix3d::Identity()-BuildSkewMatrix(w_m-gyro_bias_)*dt;//自己使用的公式
    
    Fx_.block<3,3>(INDEX_STATE_ORI,INDEX_STATE_GYRO_BIAS) =  -Eigen::Matrix3d::Identity()*dt;

    P_ = Fx_*P_*Fx_.transpose()+Fi_*Qi_*Fi_.transpose();
    return true;
}

bool ESKFQK::UpdateOdomEstimation(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, const double dt) {
    // 计算位置
    Eigen::Matrix3d R = q_.toRotationMatrix();

    pose_ += velocity_*dt+0.5*(R*(a_m-accel_bias_)+g_)*dt*dt;
    velocity_ += (R*(a_m-accel_bias_)+g_)*dt;

    Eigen::Vector3d d_theta = (w_m-gyro_bias_)*dt;
    Eigen::AngleAxisd dq(d_theta.norm(),d_theta.normalized());//!
    q_ = q_*Eigen::Quaterniond(dq);

    return true;
}



// 只将状态量置零
void ESKFQK::ResetState() {
    X_.setZero();
    // P = G'PG'^T
    Eigen::Matrix<double,DIM_STATE,DIM_STATE>  G;
    G.setIdentity();
    G.block<3,3>(6,6) -= BuildSkewMatrix(0.5*X_.block<3,1>(INDEX_STATE_ORI, 0)); 
    P_ = G*P_*G.transpose();
}

// 估计值=估计值+误差量
void ESKFQK::EliminateError() {
    pose_ +=X_.block<3,1>(INDEX_STATE_POSI,0);
    velocity_ += X_.block<3,1>(INDEX_STATE_VEL, 0);
    Eigen::Matrix3d C_nn = Sophus::SO3d::exp(X_.block<3,1>(INDEX_STATE_ORI, 0)).matrix();
    q_ = q_*Eigen::Quaterniond(C_nn); 
    accel_bias_ += X_.block<3,1>(INDEX_STATE_ACC_BIAS, 0);
    gyro_bias_ += X_.block<3,1>(INDEX_STATE_GYRO_BIAS, 0);
    g_ += X_.block<3,1>(INDEX_STATE_G,0);
}

Eigen::Matrix4d ESKFQK::GetPose() const {
    Eigen::Matrix4d pose;
    pose.block<3,3>(0,0) = q_.toRotationMatrix();
    pose.block<3,1>(0,3) = pose_;
    return pose;
}