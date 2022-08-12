//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_ESKFQK_H
#define GPS_IMU_FUSION_ESKFQK_H

#include "imu_data.h"
#include "gps_data.h"
#include "filter_interface.h"
#include "tool.h"
#include <deque>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>

// quaternion kinematics for error state kalman filter
class ESKFQK :public FilterInterface {
public:
    ESKFQK(const YAML::Node &node);

    /*!
     * 用于ESKF滤波器的初始化，设置初始位姿，初始速度
     * @param curr_gps_data 与imu时间同步的gps数据
     * @param curr_imu_data 与gps时间同步的imu数据
     * @return
     */
    bool Init(const GPSData &curr_gps_data, const IMUData &curr_imu_data);

    /*!
     * 滤波器的预测，对应卡尔曼滤波器的前两个公式
     * @param curr_imu_data
     * @return
     */
    bool Predict(const IMUData &curr_imu_data);

    /*!
     * 滤波器的矫正，对应卡尔曼滤波器的后三个公式
     * @param curr_gps_data
     * @return
     */
    bool Correct(const GPSData &curr_gps_data);

    Eigen::Matrix4d GetPose() const;

    Eigen::Vector3d GetVelocity(){
        return velocity_;
    }

private:
    void SetCovarianceV(double measurement_noise);

    void SetCovarianceQi(double v_noise, double q_noise, double a_noise, double w_noise);

    // 设置P矩阵
    void SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
                            double accel_noise, double gyro_noise, double g_noise);

    /*!
     * 通过IMU计算位姿和速度
     * @return
     */
    bool UpdateOdomEstimation(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, const double dt);

    bool UpdateErrorState(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, const double dt);

    bool ComputeAngularDelta(Eigen::Vector3d &angular_delta);

    bool ObservationOfErrorState();
    /*!
     * 对误差进行滤波之后，需要在实际算出来的轨迹中，消除这部分误差
     */
    void EliminateError();

    /*!
     * 每次矫正之后，需要重置状态变量X
     */
    void ResetState();

private:
    static const unsigned int DIM_STATE = 18; // 状态向量
    static const unsigned int DIM_STATE_NOISE = 12; // 噪声只有12维， delta v q a w
    static const unsigned int DIM_MEASUREMENT = 3; // 观测向量只有6维
    static const unsigned int DIM_MEASUREMENT_NOISE = 3; // 观测噪声

    static const unsigned int INDEX_STATE_POSI = 0; // 位置
    static const unsigned int INDEX_STATE_VEL = 3; // 速度
    static const unsigned int INDEX_STATE_ORI = 6; // 角度
    static const unsigned int INDEX_STATE_ACC_BIAS = 9; // 加速度计bias
    static const unsigned int INDEX_STATE_GYRO_BIAS = 12; // 陀螺仪bias
    static const unsigned int INDEX_STATE_G = 15; // 重力加速度状态
    static const unsigned int INDEX_MEASUREMENT_POSI = 0;

    typedef typename Eigen::Matrix<double, DIM_STATE, 1> TypeVectorX; // 状态向量
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, 1> TypeVectorY; // 观测向量 GPS的位置
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixFx; //18*18
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> TypeMatrixFi; //18*12
    typedef typename Eigen::Matrix<double, DIM_STATE_NOISE, DIM_STATE_NOISE> TypeMatrixQi;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixP;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT> TypeMatrixK;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE> TypeMatrixH;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE+1> TypeMatrixHx; // 四元数多了一维
    typedef typename Eigen::Matrix<double, DIM_STATE+1, DIM_STATE> TypeMatrixX_dx; // 四元数多了一维
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT> TypeMatrixV;

    TypeVectorX X_;
    TypeVectorY Y_;
    TypeMatrixFx Fx_;
    TypeMatrixFi Fi_;
    TypeMatrixQi Qi_;
    TypeMatrixP P_;
    TypeMatrixK K_;
    TypeMatrixH H_;
    TypeMatrixHx Hx_;
    TypeMatrixX_dx X_dx_;
    TypeMatrixV V_;

    // 初始状态
    Eigen::Vector3d init_pose_ = Eigen::Vector3d::Identity();
    Eigen::Vector3d init_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond init_q_;
    Eigen::Vector3d init_accel_bias_ = Eigen::Vector3d::Identity();
    Eigen::Vector3d init_gyro_bias_ = Eigen::Vector3d::Identity();
    Eigen::Vector3d init_g_bias_ = Eigen::Vector3d::Identity();

    // nominal state
    Eigen::Vector3d pose_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_;
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d g_;//重力加速度


    GPSData curr_gps_data_;

    double L_ = 0.0;//纬度

    std::deque<IMUData> imu_data_buff_; // 只保存两个IMU数据

public:
    // void GetFGY(TypeMatrixF& F,TypeMatrixG& G, TypeVectorY & Y);
};

#endif //GPS_IMU_FUSION_ESKF_H