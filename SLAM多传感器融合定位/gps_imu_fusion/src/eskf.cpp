//
// Created by meng on 2021/2/19.
//
#include "eskf.h"
#include "tool.h"
#include "../3rd/sophus/se3.hpp"


ESKF::ESKF(const YAML::Node &node) {

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
    double earth_rotation_speed = node["earth"]["rotation_speed"].as<double>();
    L_ = node["earth"]["latitude"].as<double>();

    double cov_prior_posi = node["ESKF"][cov_node_string]["prior"]["posi"].as<double>();
    double cov_prior_vel = node["ESKF"][cov_node_string]["prior"]["vel"].as<double>();
    double cov_prior_ori = node["ESKF"][cov_node_string]["prior"]["ori"].as<double>();
    double cov_prior_epsilon = node["ESKF"][cov_node_string]["prior"]["gyro_delta"].as<double>();
    double cov_prior_delta = node["ESKF"][cov_node_string]["prior"]["accel_delta"].as<double>();
    double cov_measurement_posi = node["ESKF"][cov_node_string]["measurement"]["posi"].as<double>();
    double cov_process_gyro = node["ESKF"][cov_node_string]["process"]["gyro_delta"].as<double>();
    double cov_process_accel = node["ESKF"][cov_node_string]["process"]["accel_delta"].as<double>();
    double cov_w_gyro = node["ESKF"][cov_node_string]["IMU_noise"]["gyro_delta"].as<double>(); // IMU数据的噪声，用来组成W矩阵
    double cov_w_accel = node["ESKF"][cov_node_string]["IMU_noise"]["accel_delta"].as<double>();
    g_ = Eigen::Vector3d(0.0, 0.0, -gravity);
    w_ = Eigen::Vector3d(0.0, earth_rotation_speed * cos(L_ * kDegree2Radian),
                         earth_rotation_speed * sin(L_ * kDegree2Radian)); // w_ie_n

    SetCovarianceP(cov_prior_posi, cov_prior_vel, cov_prior_ori,
                   cov_prior_epsilon, cov_prior_delta);
    SetCovarianceR(cov_measurement_posi);
    SetCovarianceQ(cov_process_gyro, cov_process_accel);
    SetCovarianceW(cov_w_gyro, cov_w_accel);

    X_.setZero(); // 初始化为零矩阵
    F_.setZero(); // 初始化为零矩阵
    C_.setIdentity(); // 单位矩阵
    G_.block<3,3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();

    F_.block<3,3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity(); // ?矩阵
    F_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = BuildSkewMatrix(-w_);
}

void ESKF::SetCovarianceW(double gyro_noise, double accel_noise) {
    W_.setZero();
    W_.block<3,1>(0,0) = Eigen::Vector3d(accel_noise,accel_noise,accel_noise);
    W_.block<3,1>(3,0) = Eigen::Vector3d(gyro_noise,gyro_noise,gyro_noise);
}

void ESKF::SetCovarianceQ(double gyro_noise, double accel_noise) {
    Q_.setZero();
    Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * gyro_noise * gyro_noise; // 平方
    Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * accel_noise * accel_noise;
}

void ESKF::SetCovarianceR(double posi_noise) {
    R_.setZero();
    R_ = Eigen::Matrix3d::Identity() * posi_noise * posi_noise;
}

// 设置P矩阵
void ESKF::SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
                          double gyro_noise, double accel_noise) {
    P_.setZero();
    P_.block<3,3>(INDEX_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * posi_noise;
    P_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity() * velo_noise;
    P_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix3d::Identity() * ori_noise;
    P_.block<3,3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) = Eigen::Matrix3d::Identity() * gyro_noise;
    P_.block<3,3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) = Eigen::Matrix3d::Identity() * accel_noise;
}

bool ESKF::Init(const GPSData &curr_gps_data, const IMUData &curr_imu_data) {
    init_velocity_ = curr_gps_data.true_velocity; // 用真实速度初始化
    velocity_ = init_velocity_;
    // 前右地
    Eigen::Quaterniond Q = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());
    init_pose_.block<3,3>(0,0) = Q.toRotationMatrix();
    pose_ = init_pose_;

    imu_data_buff_.clear(); // 这时ESKF中的imu数据
    imu_data_buff_.push_back(curr_imu_data);

    curr_gps_data_ = curr_gps_data;

    return true;
}

// void ESKF::GetFGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y) {
//     F = Ft_;
//     G = G_;
//     Y = Y_;
// }

bool ESKF::Correct(const GPSData &curr_gps_data) {
    curr_gps_data_ = curr_gps_data;

    Y_ = pose_.block<3,1>(0,3) - curr_gps_data.position_ned; //! Y_cal-Y_measure

    K_ = P_ * G_.transpose() * (G_ * P_ * G_.transpose() + C_ * R_ * C_.transpose()).inverse(); // kalman增益

    P_ = (TypeMatrixP::Identity() - K_ * G_) * P_;
    X_ = X_ + K_ * (Y_ - G_ * X_);

    EliminateError();

    ResetState();

    return true;
}

// IMU数据预测
bool ESKF::Predict(const IMUData &curr_imu_data) {
    imu_data_buff_.push_back(curr_imu_data);

    UpdateOdomEstimation(); // 更新 角度  速度 位置  PVQ

    double delta_t = curr_imu_data.time - imu_data_buff_.front().time; // dt

    Eigen::Vector3d curr_accel = pose_.block<3, 3>(0, 0)
                                 * curr_imu_data.linear_accel; // 导航坐标系下的加速度

    UpdateErrorState(delta_t, curr_accel); // 更新误差状态，只与R和accel相关

    imu_data_buff_.pop_front();
    return true;
}

bool ESKF::UpdateErrorState(double t, const Eigen::Vector3d &accel) {
    Eigen::Matrix3d F_23 = BuildSkewMatrix(accel);

    // 没有更新F33,因为它是常数，由地球自转和纬度决定
    F_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_ORI) = F_23;
    F_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = pose_.block<3,3>(0,0);
    F_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_GYRO_BIAS) = -pose_.block<3,3>(0,0);
    B_.setZero();
    B_.block<3,3>(INDEX_STATE_VEL, 3) = pose_.block<3,3>(0,0);
    B_.block<3,3>(INDEX_STATE_ORI, 0) = -pose_.block<3,3>(0,0);

    TypeMatrixF Fk = TypeMatrixF::Identity() + F_ * t;
    TypeMatrixB Bk = B_ * t;

    Ft_ = F_ * t;

    X_ = Fk * X_+Bk*W_;
    P_ = Fk * P_ * Fk.transpose() + Bk * Q_ * Bk.transpose();

    return true;
}

bool ESKF::UpdateOdomEstimation() {
    Eigen::Vector3d angular_delta;
    ComputeAngularDelta(angular_delta); // 平均角速度求转动过的角度，以此求delta_R

    Eigen::Matrix3d R_nm_nm_1; // i系到n系
    ComputeEarthTranform(R_nm_nm_1); // 考虑地球自传

    Eigen::Matrix3d curr_R, last_R;
    ComputeOrientation(angular_delta, R_nm_nm_1, curr_R, last_R);

    Eigen::Vector3d curr_vel, last_vel;
    ComputeVelocity(curr_vel, last_vel, curr_R, last_R);

    ComputePosition(curr_vel, last_vel);

    return true;
}

bool ESKF::ComputeAngularDelta(Eigen::Vector3d &angular_delta) {
    IMUData curr_imu_data = imu_data_buff_.at(1);
    IMUData last_imu_data = imu_data_buff_.at(0);

    double delta_t = curr_imu_data.time - last_imu_data.time;

    if (delta_t <= 0){
        return false;
    }

    Eigen::Vector3d curr_angular_vel = curr_imu_data.angle_velocity;

    Eigen::Vector3d last_angular_vel = last_imu_data.angle_velocity;

    // 直接使用last_R来对gyro_bias进行旋转
    Eigen::Matrix3d last_R = pose_.block<3, 3>(0, 0);

    Eigen::Vector3d curr_unbias_angular_vel = curr_angular_vel;
    Eigen::Vector3d last_unbias_angular_vel = last_angular_vel;

    angular_delta = 0.5 * (curr_unbias_angular_vel + last_unbias_angular_vel) * delta_t; // 中值

    return true;
}

bool ESKF::ComputeEarthTranform(Eigen::Matrix3d &R_nm_nm_1) {
    IMUData curr_imu_data = imu_data_buff_.at(1);
    IMUData last_imu_data = imu_data_buff_.at(0);

    double delta_t = curr_imu_data.time - last_imu_data.time;

    constexpr double rm = 6353346.18315;
    constexpr double rn = 6384140.52699;
    Eigen::Vector3d w_en_n(-velocity_[1] / (rm + curr_gps_data_.position_lla[2]),
                           velocity_[0] / (rn + curr_gps_data_.position_lla[2]),
                           velocity_[0] / (rn + curr_gps_data_.position_lla[2])
                           * std::tan(curr_gps_data_.position_lla[0] * kDegree2Radian));
    // 实际导航坐标系中，不动系(i系)是地心惯性系，我们需要的导航结果是相对于导航系(n系)的
    // 两个坐标系中有一个相对旋转，旋转角速度为w_in_n 。
    Eigen::Vector3d w_in_n = w_en_n + w_;  // 导航系(n系)相对于惯性系(i系)的旋转，包含导航系相对于地球的旋转和地球自转
    auto angular = delta_t * w_in_n;
    Eigen::AngleAxisd angle_axisd(angular.norm(), angular.normalized());  
    R_nm_nm_1 = angle_axisd.toRotationMatrix().transpose(); // 取转置，得到i系相对于n系的转换
    return true;
}

bool ESKF::ComputeOrientation(const Eigen::Vector3d &angular_delta,
                              const Eigen::Matrix3d R_nm_nm_1,
                              Eigen::Matrix3d &curr_R,
                              Eigen::Matrix3d &last_R) {
    Eigen::AngleAxisd angle_axisd(angular_delta.norm(), angular_delta.normalized()); // 轴角公式，前一个为转动角度，后一个为向量。角度转旋转矩阵
    last_R = pose_.block<3, 3>(0, 0);

    curr_R = R_nm_nm_1 * pose_.block<3, 3>(0, 0) * angle_axisd.toRotationMatrix(); // R*delta_R

    pose_.block<3, 3>(0, 0) = curr_R;

    return true;
}

// 使用去除重力影响和加速度bias的平均加速度计算速度
bool ESKF::ComputeVelocity(Eigen::Vector3d &curr_vel, Eigen::Vector3d& last_vel,
                                             const Eigen::Matrix3d &curr_R,
                                             const Eigen::Matrix3d last_R) {
    IMUData curr_imu_data = imu_data_buff_.at(1);
    IMUData last_imu_data = imu_data_buff_.at(0);
    double delta_t = curr_imu_data.time - last_imu_data.time;
    if (delta_t <=0 ){
        return false;
    }

    Eigen::Vector3d curr_accel = curr_imu_data.linear_accel;
    Eigen::Vector3d curr_unbias_accel = GetUnbiasAccel(curr_R * curr_accel);

    Eigen::Vector3d last_accel = last_imu_data.linear_accel;
    Eigen::Vector3d last_unbias_accel = GetUnbiasAccel(last_R * last_accel); // 减去重力影响

    last_vel = velocity_;

    velocity_ += delta_t * 0.5 * (curr_unbias_accel + last_unbias_accel);
    curr_vel = velocity_;

    return true;
}

Eigen::Vector3d ESKF::GetUnbiasAccel(const Eigen::Vector3d &accel) {
   return accel - accel_bias_ + g_; // z方向精度提高很多
    // return accel + g_;
}

bool ESKF::ComputePosition(const Eigen::Vector3d& curr_vel, const Eigen::Vector3d& last_vel){
    double delta_t = imu_data_buff_.at(1).time - imu_data_buff_.at(0).time;
    pose_.block<3,1>(0,3) += 0.5 * delta_t * (curr_vel + last_vel);

    return true;
}

// 只将状态量置零
void ESKF::ResetState() {
    X_.setZero();
}

// 估计值=估计值-误差量
void ESKF::EliminateError() {
    pose_.block<3,1>(0,3) = pose_.block<3,1>(0,3) - X_.block<3,1>(INDEX_STATE_POSI, 0);

    velocity_ = velocity_ - X_.block<3,1>(INDEX_STATE_VEL, 0);
    Eigen::Matrix3d C_nn = Sophus::SO3d::exp(X_.block<3,1>(INDEX_STATE_ORI, 0)).matrix();
    pose_.block<3,3>(0,0) = C_nn * pose_.block<3,3>(0,0); // 固定坐标系更新，左乘
    gyro_bias_ = gyro_bias_ - X_.block<3,1>(INDEX_STATE_GYRO_BIAS, 0);
    accel_bias_ = accel_bias_ - X_.block<3,1>(INDEX_STATE_ACC_BIAS, 0);
}

Eigen::Matrix4d ESKF::GetPose() const {
    return pose_;
}