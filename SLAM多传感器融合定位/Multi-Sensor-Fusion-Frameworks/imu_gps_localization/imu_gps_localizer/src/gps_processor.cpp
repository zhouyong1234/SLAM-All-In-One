#include "imu_gps_localizer/gps_processor.h"

#include "imu_gps_localizer/utils.h"
#include <iostream>
#include <opencv2/opencv.hpp>


cv::RNG rng;

double w_sigma = 10.0;

namespace ImuGpsLocalization {

GpsProcessor::GpsProcessor(const Eigen::Vector3d& I_p_Gps) : I_p_Gps_(I_p_Gps) 
{
    // file_gps_noise_.open("/home/touchair/orb_ws/src/imu_gps_localization/gps_noise.txt");

    gps_count_ = 0;
    
}

bool GpsProcessor::UpdateStateByGpsPosition(const Eigen::Vector3d& init_lla, const GpsPositionDataPtr gps_data_ptr, State* state) {
    Eigen::Matrix<double, 3, 15> H;
    Eigen::Vector3d residual;
    ComputeJacobianAndResidual(init_lla, gps_data_ptr, *state, &H, &residual);
    const Eigen::Matrix3d& V = gps_data_ptr->cov;

    // EKF.
    const Eigen::MatrixXd& P = state->cov;
    const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
    const Eigen::VectorXd delta_x = K * residual;

    // Add delta_x to state.
    AddDeltaToState(delta_x, state);

    // std::cout << "---------------------" << std::endl;

    // std::cout << "res: " << residual.transpose() << std::endl;


    // std::cout << "acc bias: " << state->acc_bias.transpose() << std::endl;
    // std::cout << "gyr bias: " << state->gyro_bias.transpose() << std::endl;
    // std::cout << "---------------------" << std::endl;

    // Covarance.
    const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    state->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
}

void GpsProcessor::ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,  
                                              const GpsPositionDataPtr gps_data, 
                                              const State& state,
                                              Eigen::Matrix<double, 3, 15>* jacobian,
                                              Eigen::Vector3d* residual) {
    const Eigen::Vector3d& G_p_I   = state.G_p_I;
    const Eigen::Matrix3d& G_R_I   = state.G_R_I;

    // Convert wgs84 to ENU frame.
    Eigen::Vector3d G_p_Gps;
    ConvertLLAToENU(init_lla, gps_data->lla, &G_p_Gps);


    // gps_count_++;
    // if(gps_count_ > 20)
    // {
    //     gps_count_ = 0;
    //     G_p_Gps[0] += rng.gaussian(w_sigma);
    //     G_p_Gps[1] += rng.gaussian(w_sigma);
    //     G_p_Gps[2] += rng.gaussian(w_sigma);
    // }



    // G_p_Gps[0] += rng.gaussian(w_sigma);
    // G_p_Gps[1] += rng.gaussian(w_sigma);
    // G_p_Gps[2] += rng.gaussian(w_sigma);


    // std::cout << G_R_I * I_p_Gps_ << std::endl;
    // Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    // matrix.block<3,1>(0,3) = Eigen::Vector3d(G_p_Gps[0], G_p_Gps[1], G_p_Gps[2]);
    // SavePose(file_gps_noise_, matrix);


    // Compute residual.
    *residual = G_p_Gps - (G_p_I + G_R_I * I_p_Gps_);

    // Compute jacobian.
    jacobian->setZero();
    jacobian->block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
    jacobian->block<3, 3>(0, 6)  = - G_R_I * GetSkewMatrix(I_p_Gps_);

    // std::cout << *jacobian << std::endl;

}



void GpsProcessor::SavePose(std::ofstream &ofs, const Eigen::Matrix4d &pose)
{
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 4; ++j)
        {
            ofs << pose(i, j);

            if(i == 2 && j == 3)
            {
                ofs << std::endl;
            }
            else
            {
                ofs << " ";
            }
        }
    }
}

void AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state) {
    state->G_p_I     += delta_x.block<3, 1>(0, 0);
    state->G_v_I     += delta_x.block<3, 1>(3, 0);
    state->acc_bias  += delta_x.block<3, 1>(9, 0);
    state->gyro_bias += delta_x.block<3, 1>(12, 0);

    if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {
        state->G_R_I *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
    }
}

}  // namespace ImuGpsLocalization