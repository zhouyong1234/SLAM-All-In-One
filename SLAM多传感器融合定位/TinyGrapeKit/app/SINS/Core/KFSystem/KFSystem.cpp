#pragma 

#include "KFSystem/KFSystem.h"
#include "InsUpdate/InsUpdater.h"
#include "ErrorModel/ErrorModel.h"
#include "MeasureModel/Measurements.h"
#include "Base/Index.h"

namespace SINS {

KFSystem::KFSystem(double angle_random_walk, double vel_random_walk) {
    imu_noise_.setZero();

    imu_noise_(0, 0) = angle_random_walk * angle_random_walk;
    imu_noise_(1, 1) = angle_random_walk * angle_random_walk;
    imu_noise_(2, 2) = angle_random_walk * angle_random_walk;

    imu_noise_(3, 3) = vel_random_walk * vel_random_walk;
    imu_noise_(4, 4) = vel_random_walk * vel_random_walk;
    imu_noise_(5, 5) = vel_random_walk * vel_random_walk;
}

bool KFSystem::ProcessImu(const ImuData::ConstPtr imu1, const ImuData::ConstPtr imu2, InsState *ins_state, KFState *kf_state) {
    double delta_t = (imu2->time - ins_state->time);

    // Insupdate
    InsUpdater::UpdateInsState(delta_t, imu1->gyro, imu2->gyro, imu1->acc, imu2->acc, ins_state);
    ins_state->time = imu2->time;

    // KF time upate.
    KFMat Phi = ErrorModel::ComputePhiMatrix(*ins_state);
    Eigen::Matrix<double, kKFStateDim, 6> G = ErrorModel::ComputeGMatrix(*ins_state);
    kf_state->kf_state = Phi * kf_state->kf_state.eval();
    kf_state->cov = Phi * kf_state->cov * Phi.transpose() + G * imu_noise_ * G.transpose() * delta_t;  
    kf_state->time = imu2->time; 

    return true;
}

template<size_t Dim>
void UpdateKFState(const Eigen::Matrix<double, Dim, 1>& zk, 
                   const Eigen::Matrix<double, Dim, kKFStateDim> &H,
                   const Eigen::Matrix<double, Dim, Dim> &R,
                   KFState *kf_state) {
    const KFMat P = kf_state->cov;

    // Update means.
    Eigen::Matrix<double, kKFStateDim, 3> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    const Eigen::Vector3d res = zk - H * kf_state->kf_state;

    kf_state->kf_state = kf_state->kf_state + K * res;

    // Update cov.
    KFMat I_KH = KFMat::Identity() - K * H;
    kf_state->cov = I_KH * P * I_KH.transpose() + K * R * K.transpose(); 
}

bool ProcessGnssVelocity(const GnssData::ConstPtr gnss_data, InsState *ins_state, KFState *kf_state) {
    Eigen::Vector3d zk;
    Eigen::Matrix<double, 3, kKFStateDim> H;
    Eigen::Matrix<double, 3, 3> H_lever_arm;

    Measurements::ComputeVelocityMeasurement(*ins_state,
                                             kf_state->gnss_arm(), 
                                             gnss_data->velocity, 
                                             &zk,
                                             &H,
                                             &H_lever_arm);
    H.block<3, 3>(0, kGnssArmIdx) = H_lever_arm;

    UpdateKFState<3>(zk, H, gnss_data->velocity.cwiseAbs2().asDiagonal(), kf_state);
    
    return true;
}

bool ProcessGnssPosition(const GnssData::ConstPtr gnss_data, InsState *ins_state, KFState *kf_state) {
    Eigen::Vector3d zk;
    Eigen::Matrix<double, 3, kKFStateDim> H;
    Eigen::Matrix<double, 3, 3> H_lever_arm;

    const Eigen::Matrix3d Mpv = ComputeMpv(ins_state->Rm + ins_state->lat_lon_hei[2], 
                                           ins_state->Rn + ins_state->lat_lon_hei[2], 
                                           ins_state->lat_lon_hei[0]);

    Measurements::ComputePostionMeasurement(*ins_state, 
                                            Mpv,
                                            kf_state->gnss_arm(), 
                                            gnss_data->lat_lon_hei, 
                                            &zk,
                                            &H,
                                            &H_lever_arm);
    H.block<3, 3>(0, kGnssArmIdx) = H_lever_arm;

    UpdateKFState<3>(zk, H, gnss_data->lat_lon_hei_std.cwiseAbs2().asDiagonal(), kf_state);
    
    return true;
}


}  // namespace SINS