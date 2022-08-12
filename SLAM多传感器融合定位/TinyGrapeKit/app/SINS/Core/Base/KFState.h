#pragma once

#include <Eigen/Eigen>
#include "Base/Index.h"

namespace SINS {

struct KFState {
    double time = -1.0;

    /******* Mean ********/
    Eigen::Matrix<double, kKFStateDim, 1> kf_state;

    // AVP error.
    Eigen::Vector3d att_err() { return kf_state.block<kAttErrDim, 1>(kAttErrIdx, 0); }
    Eigen::Vector3d vel_err() { return kf_state.block<kVelErrDim, 1>(kVelErrIdx, 0); }
    Eigen::Vector3d pos_err() { return kf_state.block<kPosErrDim, 1>(kPosErrIdx, 0); }

    // IMU bias error.
    Eigen::Vector3d gyro_bias_err()  { return kf_state.block<kGyroBiasErrDim, 1>(kGyroBiasErrIdx, 0); }
    Eigen::Vector3d acc_bias_err()  { return kf_state.block<kAccBiasErrDim, 1>(kAccBiasErrIdx, 0); }

    // GNSS extrinsic.
    Eigen::Vector3d gnss_arm() { return kf_state.block<kGnssArmDim, 1>(kGnssArmIdx, 0); }

    /******* Covariance ********/
    KFMat cov;
};

}  // namespace SINS