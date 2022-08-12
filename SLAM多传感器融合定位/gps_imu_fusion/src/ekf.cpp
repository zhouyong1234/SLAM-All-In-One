//
// Created by meng on 2021/2/19.
//
#include "ekf.h"
#include "../3rd/sophus/se3.hpp"

constexpr double kDegree2Radian = M_PI / 180.0;

EKF::EKF(const YAML::Node &node) {
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

    double cov_prior_posi = node["EKF"][cov_node_string]["prior"]["posi"].as<double>();
    double cov_prior_vel = node["EKF"][cov_node_string]["prior"]["vel"].as<double>();
    double cov_prior_ori = node["EKF"][cov_node_string]["prior"]["ori"].as<double>();
    double cov_prior_epsilon = node["EKF"][cov_node_string]["prior"]["gyro_delta"].as<double>();
    double cov_prior_delta = node["EKF"][cov_node_string]["prior"]["accel_delta"].as<double>();
    double cov_measurement_posi = node["EKF"][cov_node_string]["measurement"]["posi"].as<double>();
    double cov_process_gyro = node["EKF"][cov_node_string]["process"]["gyro_delta"].as<double>();
    double cov_process_accel = node["EKF"][cov_node_string]["process"]["accel_delta"].as<double>();

    double cov_w_gyro = node["EKF"][cov_node_string]["IMU_noise"]["gyro_delta"].as<double>();
    double cov_w_accel = node["EKF"][cov_node_string]["IMU_noise"]["accel_delta"].as<double>();

    SetCovarianceP(cov_prior_posi, cov_prior_vel, cov_prior_ori,
                   cov_prior_epsilon, cov_prior_delta);
    SetCovarianceR(cov_measurement_posi);
    SetCovarianceQ(cov_process_gyro, cov_process_accel);
    SetCovarianceW(cov_w_gyro, cov_w_accel);

    gt_.setZero();
    gt_.block<3,1>(INDEX_STATE_VEL,0) = Eigen::Vector3d(0, 0, -gravity);
}

void EKF::SetCovarianceW(double gyro_w_noise, double accel_w_noise) {
    W_.setZero();
    W_.block<3,1>(0,0) = Eigen::Vector3d(gyro_w_noise,gyro_w_noise,gyro_w_noise);
    W_.block<3,1>(3,0) = Eigen::Vector3d(accel_w_noise,accel_w_noise,accel_w_noise);
}

void EKF::SetCovarianceQ(double gyro_noise_cov, double accel_noise_cov) {
    Q_.setZero();
    Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * gyro_noise_cov * gyro_noise_cov; // 平方
    Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * accel_noise_cov * accel_noise_cov;
}

void EKF::SetCovarianceR(double posi_noise) {
    R_.setZero();
    R_ = Eigen::Matrix3d::Identity() * posi_noise * posi_noise;
}

// 设置P矩阵
void EKF::SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
                          double gyro_noise, double accel_noise) {
    P_.setZero();
    P_.block<3,3>(INDEX_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * posi_noise;
    P_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity() * velo_noise;
    P_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix3d::Identity() * ori_noise;
    P_.block<3,3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) = Eigen::Matrix3d::Identity() * gyro_noise;
    P_.block<3,3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) = Eigen::Matrix3d::Identity() * accel_noise;
}

bool EKF::Init(const GPSData &curr_gps_data, const IMUData &curr_imu_data) {

    imu_data_buff_.clear(); // 这时EKF中的imu数据
    imu_data_buff_.push_back(curr_imu_data);

    curr_gps_data_ = curr_gps_data;

    X_.setZero(); 
    X_.block<3,1>(INDEX_STATE_POSI,0) =curr_gps_data.position_ned;
    X_.block<3,1>(INDEX_STATE_VEL,0) = curr_gps_data.true_velocity; // 用真实速度初始化

    // 前右地
    Eigen::Quaterniond Q = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());
    X_(INDEX_STATE_ORI+0,0) = Q.w();
    X_(INDEX_STATE_ORI+1,0) = Q.x();
    X_(INDEX_STATE_ORI+2,0) = Q.y();
    X_(INDEX_STATE_ORI+3,0) = Q.z();

    return true;
}

// void EKF::GetFGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y) {
//     F = Ft_;
//     G = G_;
//     Y = Y_;
// }

bool EKF::Correct(const GPSData &curr_gps_data) {
    curr_gps_data_ = curr_gps_data;

    C_.setIdentity(); // 单位矩阵
    G_.setZero();
    G_.block<3,3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();

    Y_ = curr_gps_data.position_ned; //Y_measure
    K_ = P_ * G_.transpose() * (G_ * P_ * G_.transpose() + C_ * R_ * C_.transpose()).inverse(); // kalman增益

    P_ = (TypeMatrixP::Identity() - K_ * G_) * P_;
    // P_ = (TypeMatrixP::Identity() - K_ * G_) * P_ * (TypeMatrixP::Identity() - K_ * G_).transpose()+K_*C_*K_.transpose();
    X_ = X_ + K_ * (Y_ - G_ * X_);

    UpdateState();
    return true;
}

// IMU数据预测
bool EKF::Predict(const IMUData &curr_imu_data) {
    imu_data_buff_.push_back(curr_imu_data);

    double delta_t = curr_imu_data.time - imu_data_buff_.front().time; // dt

    Eigen::Vector3d curr_accel = curr_imu_data.linear_accel; // 局部坐标系下加速度
    Eigen::Vector3d curr_angle_velocity = curr_imu_data.angle_velocity; // 局部坐标系下的角速度
    Eigen::Quaterniond curr_ori = Eigen::Quaterniond(pose_.block<3, 3>(0, 0));
    UpdateEkfState(delta_t, curr_accel,curr_angle_velocity,curr_ori);
    UpdateState(); //! 每次都需要更新位姿
    imu_data_buff_.pop_front();
    return true;
}

bool EKF::UpdateEkfState(const double t, const Eigen::Vector3d &accel, const Eigen::Vector3d& curr_angle_velocity,
                        const Eigen::Quaterniond& curr_ori ) {
    F_.setZero(); // 初始化为零矩阵
    F_.block<3,3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity(); 
    
    double q0 = curr_ori.w();
    double q1 = curr_ori.x();
    double q2 = curr_ori.y();
    double q3 = curr_ori.z();
    double FVq0 = 2 * Eigen::Vector3d(q0,-q3,q2).transpose()*accel;
    double FVq1 = 2 * Eigen::Vector3d(q1,q2,q3).transpose()*accel;
    double FVq2 = 2 * Eigen::Vector3d(-q2,q1,q0).transpose()*accel;
    double FVq3 = 2 * Eigen::Vector3d(-q3,-q0,q1).transpose()*accel;
    Eigen::Matrix<double,3,4> FVq = (Eigen::Matrix<double,3,4>()<<  FVq0,FVq1,FVq2,FVq3,
                                                                    -FVq3,-FVq2,FVq1,FVq0,
                                                                    FVq2,-FVq3,-FVq0,FVq1).finished();

    F_.block<3,4>(INDEX_STATE_VEL, INDEX_STATE_ORI) = FVq;
    F_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = pose_.block<3,3>(0,0);

    Eigen::Vector3d w = curr_angle_velocity;
    Eigen::Matrix<double,4,4> Fqq = 0.5* (Eigen::Matrix<double,4,4>()<<0,-w.x(),-w.y(),-w.z(),
                                                                        w.x(),0,w.z(),-w.y(),
                                                                        w.y(),-w.z(),0,w.x(),
                                                                        w.z(),w.y(),-w.x(),0).finished();
    F_.block<4,4>(INDEX_STATE_ORI,INDEX_STATE_ORI) = Fqq;

    Eigen::Matrix<double,4,3> Fqkesi  = 0.5 * (Eigen::Matrix<double,4,3>()<<-q1,-q2,-q3,
                                                                        q0,-q3,q2,
                                                                        q3,q0,-q1,
                                                                        -q2,q1,q0).finished();
    F_.block<4,3>(INDEX_STATE_ORI,INDEX_STATE_GYRO_BIAS) = Fqkesi;

    B_.setZero();
    B_.block<3,3>(INDEX_STATE_VEL, 3) = pose_.block<3,3>(0,0);
    B_.block<4,3>(INDEX_STATE_ORI, 0) = Fqkesi;

    TypeMatrixF Fk = TypeMatrixF::Identity() + F_ * t;
    TypeMatrixB Bk = B_ * t;

    X_ = Fk * X_ + Bk * W_+ gt_*t ;  //现象：不加W_和gt_两项时，效果更好。
    P_ = Fk * P_ * Fk.transpose() + Bk * Q_ * Bk.transpose();

    return true;
}


void EKF::UpdateState() {
    pose_.block<3,1>(0,3) =  X_.block<3,1>(INDEX_STATE_POSI, 0);

    Eigen::Quaterniond q;
    q.w() = X_(INDEX_STATE_ORI + 0, 0);
    q.x() = X_(INDEX_STATE_ORI + 1, 0);
    q.y() = X_(INDEX_STATE_ORI + 2, 0);
    q.z() = X_(INDEX_STATE_ORI + 3, 0);
    q.normalize();

    // 修改旋转矩阵
    pose_.block<3,3>(0,0) = q.toRotationMatrix();
}

Eigen::Matrix4d EKF::GetPose() const {
    return pose_;
}