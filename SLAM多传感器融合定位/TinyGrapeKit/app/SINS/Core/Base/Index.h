#pragma once

#include <Eigen/Eigen>

namespace SINS {

constexpr size_t kAttErrIdx = 0u;
constexpr size_t kAttErrDim = 3u;

constexpr size_t kVelErrIdx = kAttErrIdx + kAttErrDim;
constexpr size_t kVelErrDim = 3u;

constexpr size_t kPosErrIdx = kVelErrIdx + kVelErrDim;
constexpr size_t kPosErrDim = 3u;

constexpr size_t kGyroBiasErrIdx = kPosErrIdx + kPosErrDim;
constexpr size_t kGyroBiasErrDim = 3u;

constexpr size_t kAccBiasErrIdx = kGyroBiasErrIdx + kGyroBiasErrDim;
constexpr size_t kAccBiasErrDim = 3u;

constexpr size_t kGnssArmIdx = kAccBiasErrIdx + kAccBiasErrDim;
constexpr size_t kGnssArmDim = 3u;

constexpr size_t kKFStateDim = kAttErrDim + kVelErrDim + kPosErrDim + kGyroBiasErrDim + kAccBiasErrDim + kGnssArmDim;

constexpr size_t kGyroNoiseIdx = 0u;
constexpr size_t kAccNoiseIdx = 3u;

using KFMat = Eigen::Matrix<double, kKFStateDim, kKFStateDim>;

}  // namespace SINS