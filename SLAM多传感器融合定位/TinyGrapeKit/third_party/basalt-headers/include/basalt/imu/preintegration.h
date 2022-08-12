/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt-headers.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@file
@brief IMU preintegration
*/

#pragma once

#include <basalt/imu/imu_types.h>
#include <basalt/utils/assert.h>
#include <basalt/utils/sophus_utils.hpp>

namespace basalt {

/// @brief Integrated pseudo-measurement that combines several consecutive IMU
/// measurements.
class IntegratedImuMeasurement {
 public:
  using Ptr = std::shared_ptr<IntegratedImuMeasurement>;

  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using VecN = Eigen::Matrix<double, POSE_VEL_SIZE, 1>;
  using MatNN = Eigen::Matrix<double, POSE_VEL_SIZE, POSE_VEL_SIZE>;
  using MatN3 = Eigen::Matrix<double, POSE_VEL_SIZE, 3>;
  using MatN6 = Eigen::Matrix<double, POSE_VEL_SIZE, 6>;

  /// @brief Propagate current state given ImuData and optionally compute
  /// Jacobians.
  ///
  /// @param[in] curr_state current state
  /// @param[in] data IMU data
  /// @param[out] next_state predicted state
  /// @param[out] d_next_d_curr Jacobian of the predicted state with respect
  /// to current state
  /// @param[out] d_next_d_accel Jacobian of the predicted state with respect
  /// accelerometer measurement
  /// @param[out] d_next_d_gyro Jacobian of the predicted state with respect
  /// gyroscope measurement
  inline static void propagateState(const PoseVelState& curr_state,
                                    const ImuData& data,
                                    PoseVelState& next_state,
                                    MatNN* d_next_d_curr = nullptr,
                                    MatN3* d_next_d_accel = nullptr,
                                    MatN3* d_next_d_gyro = nullptr) {
    BASALT_ASSERT_STREAM(
        data.t_ns > curr_state.t_ns,
        "data.t_ns " << data.t_ns << " curr_state.t_ns " << curr_state.t_ns);

    int64_t dt_ns = data.t_ns - curr_state.t_ns;
    double dt = dt_ns * 1e-9;

    Sophus::SO3d R_w_i_new_2 =
        curr_state.T_w_i.so3() * Sophus::SO3d::exp(0.5 * dt * data.gyro);
    Eigen::Matrix3d RR_w_i_new_2 = R_w_i_new_2.matrix();

    Eigen::Vector3d accel_world = RR_w_i_new_2 * data.accel;

    next_state.t_ns = data.t_ns;
    next_state.T_w_i.so3() =
        curr_state.T_w_i.so3() * Sophus::SO3d::exp(dt * data.gyro);
    next_state.vel_w_i = curr_state.vel_w_i + accel_world * dt;
    next_state.T_w_i.translation() = curr_state.T_w_i.translation() +
                                     curr_state.vel_w_i * dt +
                                     0.5 * accel_world * dt * dt;

    if (d_next_d_curr) {
      d_next_d_curr->setIdentity();
      d_next_d_curr->block<3, 3>(0, 6).diagonal().setConstant(dt);
      d_next_d_curr->block<3, 3>(6, 3) = Sophus::SO3d::hat(-accel_world * dt);
      d_next_d_curr->block<3, 3>(0, 3) =
          d_next_d_curr->block<3, 3>(6, 3) * dt * 0.5;
    }

    if (d_next_d_accel) {
      d_next_d_accel->setZero();
      d_next_d_accel->block<3, 3>(0, 0) = 0.5 * RR_w_i_new_2 * dt * dt;
      d_next_d_accel->block<3, 3>(6, 0) = RR_w_i_new_2 * dt;
    }

    if (d_next_d_gyro) {
      d_next_d_gyro->setZero();

      Eigen::Matrix3d Jr;
      Sophus::rightJacobianSO3(dt * data.gyro, Jr);

      Eigen::Matrix3d Jr2;
      Sophus::rightJacobianSO3(0.5 * dt * data.gyro, Jr2);

      d_next_d_gyro->block<3, 3>(3, 0) =
          next_state.T_w_i.so3().matrix() * Jr * dt;
      d_next_d_gyro->block<3, 3>(6, 0) =
          Sophus::SO3d::hat(-accel_world * dt) * RR_w_i_new_2 * Jr2 * 0.5 * dt;

      d_next_d_gyro->block<3, 3>(0, 0) =
          0.5 * dt * d_next_d_gyro->block<3, 3>(6, 0);
    }
  }

  /// @brief Default constructor.
  IntegratedImuMeasurement() : start_t_ns(0), cov_inv_computed(false) {
    cov.setZero();
    d_state_d_ba.setZero();
    d_state_d_bg.setZero();
    bias_gyro_lin.setZero();
    bias_accel_lin.setZero();
  }

  /// @brief Constructor with start time and bias estimates.
  IntegratedImuMeasurement(int64_t start_t_ns,
                           const Eigen::Vector3d& bias_gyro_lin,
                           const Eigen::Vector3d& bias_accel_lin)
      : start_t_ns(start_t_ns),
        cov_inv_computed(false),
        bias_gyro_lin(bias_gyro_lin),
        bias_accel_lin(bias_accel_lin) {
    cov.setZero();
    d_state_d_ba.setZero();
    d_state_d_bg.setZero();
  }

  /// @brief Integrate IMU data
  ///
  /// @param[in] data IMU data
  /// @param[in] accel_cov diagonal of accelerometer noise covariance matrix
  /// @param[in] gyro_cov diagonal of gyroscope noise covariance matrix
  void integrate(const ImuData& data, const Vec3& accel_cov,
                 const Vec3& gyro_cov) {
    ImuData data_corrected = data;
    data_corrected.t_ns -= start_t_ns;
    data_corrected.accel -= bias_accel_lin;
    data_corrected.gyro -= bias_gyro_lin;

    PoseVelState new_state;

    MatNN F;
    MatN3 A, G;

    propagateState(delta_state, data_corrected, new_state, &F, &A, &G);

    delta_state = new_state;
    cov = F * cov * F.transpose() + A * accel_cov.asDiagonal() * A.transpose() +
          G * gyro_cov.asDiagonal() * G.transpose();
    cov_inv_computed = false;

    d_state_d_ba = -A + F * d_state_d_ba;
    d_state_d_bg = -G + F * d_state_d_bg;
  }

  /// @brief Predict state given this pseudo-measurement
  ///
  /// @param[in] state0 current state
  /// @param[in] g gravity vector
  /// @param[out] state1 predicted state
  void predictState(const PoseVelState& state0, const Eigen::Vector3d& g,
                    PoseVelState& state1) const {
    double dt = delta_state.t_ns * 1e-9;

    state1.T_w_i.so3() = state0.T_w_i.so3() * delta_state.T_w_i.so3();
    state1.vel_w_i =
        state0.vel_w_i + g * dt + state0.T_w_i.so3() * delta_state.vel_w_i;
    state1.T_w_i.translation() =
        state0.T_w_i.translation() + state0.vel_w_i * dt + 0.5 * g * dt * dt +
        state0.T_w_i.so3() * delta_state.T_w_i.translation();
  }

  /// @brief Compute residual between two states given this pseudo-measurement
  /// and optionally compute Jacobians.
  ///
  /// @param[in] state0 initial state
  /// @param[in] g gravity vector
  /// @param[in] state1 next state
  /// @param[in] curr_bg current estimate of gyroscope bias
  /// @param[in] curr_ba current estimate of accelerometer bias
  /// @param[out] d_res_d_state0 if not nullptr, Jacobian of the residual with
  /// respect to state0
  /// @param[out] d_res_d_state1 if not nullptr, Jacobian of the residual with
  /// respect to state1
  /// @param[out] d_res_d_bg if not nullptr, Jacobian of the residual with
  /// respect to gyroscope bias
  /// @param[out] d_res_d_ba if not nullptr, Jacobian of the residual with
  /// respect to accelerometer bias
  /// @return residual
  VecN residual(const PoseVelState& state0, const Eigen::Vector3d& g,
                const PoseVelState& state1, const Eigen::Vector3d& curr_bg,
                const Eigen::Vector3d& curr_ba, MatNN* d_res_d_state0 = nullptr,
                MatNN* d_res_d_state1 = nullptr, MatN3* d_res_d_bg = nullptr,
                MatN3* d_res_d_ba = nullptr) const {
    double dt = delta_state.t_ns * 1e-9;
    VecN res;

    VecN bg_diff, ba_diff;
    bg_diff = d_state_d_bg * (curr_bg - bias_gyro_lin);
    ba_diff = d_state_d_ba * (curr_ba - bias_accel_lin);

    BASALT_ASSERT(ba_diff.segment<3>(3).isApproxToConstant(0));

    Eigen::Matrix3d R0_inv = state0.T_w_i.so3().inverse().matrix();
    Eigen::Vector3d tmp =
        R0_inv * (state1.T_w_i.translation() - state0.T_w_i.translation() -
                  state0.vel_w_i * dt - 0.5 * g * dt * dt);

    res.segment<3>(0) = tmp - (delta_state.T_w_i.translation() +
                               bg_diff.segment<3>(0) + ba_diff.segment<3>(0));
    res.segment<3>(3) =
        (Sophus::SO3d::exp(bg_diff.segment<3>(3)) * delta_state.T_w_i.so3() *
         state1.T_w_i.so3().inverse() * state0.T_w_i.so3())
            .log();

    Eigen::Vector3d tmp2 = R0_inv * (state1.vel_w_i - state0.vel_w_i - g * dt);
    res.segment<3>(6) = tmp2 - (delta_state.vel_w_i + bg_diff.segment<3>(6) +
                                ba_diff.segment<3>(6));

    if (d_res_d_state0 || d_res_d_state1) {
      Eigen::Matrix3d J;
      Sophus::rightJacobianInvSO3(res.segment<3>(3), J);

      if (d_res_d_state0) {
        d_res_d_state0->setZero();
        d_res_d_state0->block<3, 3>(0, 0) = -R0_inv;
        d_res_d_state0->block<3, 3>(0, 3) = Sophus::SO3d::hat(tmp) * R0_inv;
        d_res_d_state0->block<3, 3>(3, 3) = J * R0_inv;
        d_res_d_state0->block<3, 3>(6, 3) = Sophus::SO3d::hat(tmp2) * R0_inv;

        d_res_d_state0->block<3, 3>(0, 6) = -R0_inv * dt;
        d_res_d_state0->block<3, 3>(6, 6) = -R0_inv;
      }

      if (d_res_d_state1) {
        d_res_d_state1->setZero();
        d_res_d_state1->block<3, 3>(0, 0) = R0_inv;
        d_res_d_state1->block<3, 3>(3, 3) = -J * R0_inv;

        d_res_d_state1->block<3, 3>(6, 6) = R0_inv;
      }
    }

    if (d_res_d_ba) {
      *d_res_d_ba = -d_state_d_ba;
    }

    if (d_res_d_bg) {
      d_res_d_bg->setZero();
      *d_res_d_bg = -d_state_d_bg;

      Eigen::Matrix3d J;
      Sophus::leftJacobianInvSO3(res.segment<3>(3), J);
      d_res_d_bg->block<3, 3>(3, 0) = J * d_state_d_bg.block<3, 3>(3, 0);
    }

    return res;
  }

  /// @brief Time duretion of preintegrated measurement in nanoseconds.
  int64_t get_dt_ns() const { return delta_state.t_ns; }

  /// @brief Start time of preintegrated measurement in nanoseconds.
  int64_t get_start_t_ns() const { return start_t_ns; }

  /// @brief Inverse of the measurement covariance matrix
  inline const MatNN& get_cov_inv() const {
    if (!cov_inv_computed) {
      cov_inv.setIdentity();
      cov.ldlt().solveInPlace(cov_inv);
      cov_inv_computed = true;
    }

    return cov_inv;
  }

  /// @brief Measurement covariance matrix
  const MatNN& get_cov() const { return cov; }

  // Just for testing...
  /// @brief Delta state
  const PoseVelState& getDeltaState() const { return delta_state; }

  /// @brief Jacobian of delta state with respect to accelerometer bias
  const MatN3& get_d_state_d_ba() const { return d_state_d_ba; }

  /// @brief Jacobian of delta state with respect to gyroscope bias
  const MatN3& get_d_state_d_bg() const { return d_state_d_bg; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  int64_t start_t_ns;  ///< Integration start time in nanoseconds

  PoseVelState delta_state;  ///< Delta state

  MatNN cov;              ///< Measurement covariance
  mutable MatNN cov_inv;  ///< Cached inverse of measurement covariance
  mutable bool
      cov_inv_computed;  ///< If the cached inverse covariance is computed

  MatN3 d_state_d_ba, d_state_d_bg;

  Eigen::Vector3d bias_gyro_lin, bias_accel_lin;
};

}  // namespace basalt
