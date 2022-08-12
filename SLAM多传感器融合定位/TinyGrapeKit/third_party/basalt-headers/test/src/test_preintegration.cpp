/**
BSD 3-Clause License

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
*/

#include <basalt/imu/preintegration.h>
#include <basalt/spline/se3_spline.h>

#include <iostream>

#include "gtest/gtest.h"
#include "test_utils.h"

namespace basalt {
namespace constants {
static const Eigen::Vector3d g(0, 0, -9.81);
}
}  // namespace basalt

static const double accel_std_dev = 0.23;
static const double gyro_std_dev = 0.0027;

// Smaller noise for testing
// static const double accel_std_dev = 0.00023;
// static const double gyro_std_dev = 0.0000027;

std::random_device rd{};
std::mt19937 gen{rd()};

std::normal_distribution<> gyro_noise_dist{0, gyro_std_dev};
std::normal_distribution<> accel_noise_dist{0, accel_std_dev};

TEST(ImuPreintegrationTestCase, PredictTestGT) {
  int num_knots = 15;

  basalt::IntegratedImuMeasurement imu_meas(0, Eigen::Vector3d::Zero(),
                                            Eigen::Vector3d::Zero());

  basalt::Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  basalt::PoseVelState state0, state1, state1_gt;

  state0.T_w_i = gt_spline.pose(int64_t(0));
  state0.vel_w_i = gt_spline.transVelWorld(int64_t(0));

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(20e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - basalt::constants::g);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    basalt::ImuData data;
    data.accel = accel_body;
    data.gyro = rot_vel_body;
    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    imu_meas.integrate(data, Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
  }

  state1_gt.T_w_i = gt_spline.pose(imu_meas.get_dt_ns());
  state1_gt.vel_w_i = gt_spline.transVelWorld(imu_meas.get_dt_ns());

  imu_meas.predictState(state0, basalt::constants::g, state1);

  EXPECT_TRUE(state1_gt.vel_w_i.isApprox(state1.vel_w_i, 1e-4))
      << "vel1_gt " << state1_gt.vel_w_i.transpose() << " vel1 "
      << state1.vel_w_i.transpose();

  EXPECT_LE(state1_gt.T_w_i.unit_quaternion().angularDistance(
                state1.T_w_i.unit_quaternion()),
            1e-6);

  EXPECT_TRUE(
      state1_gt.T_w_i.translation().isApprox(state1.T_w_i.translation(), 1e-4))
      << "pose1_gt p " << state1_gt.T_w_i.translation().transpose()
      << " pose1 p " << state1.T_w_i.translation().transpose();
}

TEST(ImuPreintegrationTestCase, PredictTest) {
  int num_knots = 15;

  basalt::Se3Spline<5> gt_spline(int64_t(2e9));
  gt_spline.genRandomTrajectory(num_knots);

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2; t_ns < gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - basalt::constants::g);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    basalt::ImuData data;
    data.accel = accel_body;
    data.gyro = rot_vel_body;
    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    basalt::PoseVelState next_state;

    int64_t curr_state_t_ns = t_ns - dt_ns / 2;
    basalt::PoseVelState curr_state(curr_state_t_ns,
                                    gt_spline.pose(curr_state_t_ns),
                                    gt_spline.transVelWorld(curr_state_t_ns));

    basalt::IntegratedImuMeasurement::MatNN F;
    basalt::IntegratedImuMeasurement::MatN3 A, G;

    basalt::IntegratedImuMeasurement::propagateState(curr_state, data,
                                                     next_state, &F, &A, &G);

    {
      basalt::PoseVelState::VecN x0;
      x0.setZero();
      test_jacobian(
          "F_TEST", F,
          [&](const basalt::PoseVelState::VecN& x) {
            basalt::PoseVelState curr_state1 = curr_state;
            curr_state1.applyInc(x);
            basalt::PoseVelState next_state1;

            basalt::IntegratedImuMeasurement::propagateState(curr_state1, data,
                                                             next_state1);

            return next_state.diff(next_state1);
          },
          x0);
    }

    {
      Eigen::Vector3d x0;
      x0.setZero();
      test_jacobian(
          "A_TEST", A,
          [&](const Eigen::Vector3d& x) {
            basalt::ImuData data1 = data;
            data1.accel += x;
            basalt::PoseVelState next_state1;

            basalt::IntegratedImuMeasurement::propagateState(curr_state, data1,
                                                             next_state1);

            return next_state.diff(next_state1);
          },
          x0);
    }

    {
      Eigen::Vector3d x0;
      x0.setZero();
      test_jacobian(
          "G_TEST", G,
          [&](const Eigen::Vector3d& x) {
            basalt::ImuData data1 = data;
            data1.gyro += x;
            basalt::PoseVelState next_state1;

            basalt::IntegratedImuMeasurement::propagateState(curr_state, data1,
                                                             next_state1);

            return next_state.diff(next_state1);
          },
          x0, 1e-8);
    }
  }
}

TEST(ImuPreintegrationTestCase, ResidualTest) {
  int num_knots = 15;

  Eigen::Vector3d bg, ba;
  bg = Eigen::Vector3d::Random() / 100;
  ba = Eigen::Vector3d::Random() / 10;

  basalt::IntegratedImuMeasurement imu_meas(0, bg, ba);

  basalt::Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  basalt::PoseVelState state0, state1, state1_gt;

  state0.T_w_i = gt_spline.pose(int64_t(0));
  state0.vel_w_i = gt_spline.transVelWorld(int64_t(0));

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(1e8);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - basalt::constants::g);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    basalt::ImuData data;
    data.accel = accel_body + ba;
    data.gyro = rot_vel_body + bg;
    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    imu_meas.integrate(data, Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
  }

  state1_gt.T_w_i = gt_spline.pose(imu_meas.get_dt_ns());
  state1_gt.vel_w_i = gt_spline.transVelWorld(imu_meas.get_dt_ns());

  basalt::PoseVelState::VecN res_gt =
      imu_meas.residual(state0, basalt::constants::g, state1_gt, bg, ba);

  EXPECT_LE(res_gt.array().abs().maxCoeff(), 1e-6)
      << "res_gt " << res_gt.transpose();

  state1.T_w_i = gt_spline.pose(imu_meas.get_dt_ns()) *
                 Sophus::se3_expd(Sophus::Vector6d::Random() / 10);
  state1.vel_w_i = gt_spline.transVelWorld(imu_meas.get_dt_ns()) +
                   Sophus::Vector3d::Random() / 10;

  basalt::IntegratedImuMeasurement::MatNN d_res_d_state0, d_res_d_state1;
  basalt::IntegratedImuMeasurement::MatN3 d_res_d_bg, d_res_d_ba;

  imu_meas.residual(state0, basalt::constants::g, state1, bg, ba,
                    &d_res_d_state0, &d_res_d_state1, &d_res_d_bg, &d_res_d_ba);

  {
    basalt::PoseVelState::VecN x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_state0", d_res_d_state0,
        [&](const basalt::PoseVelState::VecN& x) {
          basalt::PoseVelState state0_new = state0;
          state0_new.applyInc(x);

          return imu_meas.residual(state0_new, basalt::constants::g, state1, bg,
                                   ba);
        },
        x0);
  }

  {
    basalt::PoseVelState::VecN x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_state1", d_res_d_state1,
        [&](const basalt::PoseVelState::VecN& x) {
          basalt::PoseVelState state1_new = state1;
          state1_new.applyInc(x);

          return imu_meas.residual(state0, basalt::constants::g, state1_new, bg,
                                   ba);
        },
        x0);
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_bg", d_res_d_bg,
        [&](const Sophus::Vector3d& x) {
          return imu_meas.residual(state0, basalt::constants::g, state1, bg + x,
                                   ba);
        },
        x0);
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_ba", d_res_d_ba,
        [&](const Sophus::Vector3d& x) {
          return imu_meas.residual(state0, basalt::constants::g, state1, bg,
                                   ba + x);
        },
        x0);
  }
}

TEST(ImuPreintegrationTestCase, BiasTest) {
  int num_knots = 15;

  Eigen::Vector3d bg, ba;
  bg = Eigen::Vector3d::Random() / 100;
  ba = Eigen::Vector3d::Random() / 10;

  basalt::Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  Eigen::aligned_vector<Eigen::Vector3d> accel_data_vec;
  Eigen::aligned_vector<Eigen::Vector3d> gyro_data_vec;
  Eigen::aligned_vector<int64_t> timestamps_vec;

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(1e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - basalt::constants::g);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    accel_data_vec.emplace_back(accel_body + ba);
    gyro_data_vec.emplace_back(rot_vel_body + bg);
    timestamps_vec.emplace_back(t_ns + dt_ns / 2);
  }

  basalt::IntegratedImuMeasurement imu_meas(0, bg, ba);

  for (size_t i = 0; i < timestamps_vec.size(); i++) {
    basalt::ImuData data;
    data.accel = accel_data_vec[i];
    data.gyro = gyro_data_vec[i];
    data.t_ns = timestamps_vec[i];

    imu_meas.integrate(data, Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
  }

  basalt::IntegratedImuMeasurement::MatN3 d_res_d_ba, d_res_d_bg;

  basalt::PoseVelState delta_state = imu_meas.getDeltaState();

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_bg", imu_meas.get_d_state_d_bg(),
        [&](const Sophus::Vector3d& x) {
          basalt::IntegratedImuMeasurement imu_meas1(0, bg + x, ba);

          for (size_t i = 0; i < timestamps_vec.size(); i++) {
            basalt::ImuData data;
            data.accel = accel_data_vec[i];
            data.gyro = gyro_data_vec[i];
            data.t_ns = timestamps_vec[i];

            imu_meas1.integrate(data, Eigen::Vector3d::Ones(),
                                Eigen::Vector3d::Ones());
          }

          basalt::PoseVelState delta_state1 = imu_meas1.getDeltaState();
          return delta_state.diff(delta_state1);
        },
        x0);
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_ba", imu_meas.get_d_state_d_ba(),
        [&](const Sophus::Vector3d& x) {
          basalt::IntegratedImuMeasurement imu_meas1(0, bg, ba + x);

          for (size_t i = 0; i < timestamps_vec.size(); i++) {
            basalt::ImuData data;
            data.accel = accel_data_vec[i];
            data.gyro = gyro_data_vec[i];
            data.t_ns = timestamps_vec[i];

            imu_meas1.integrate(data, Eigen::Vector3d::Ones(),
                                Eigen::Vector3d::Ones());
          }

          basalt::PoseVelState delta_state1 = imu_meas1.getDeltaState();
          return delta_state.diff(delta_state1);
        },
        x0);
  }
}

TEST(ImuPreintegrationTestCase, ResidualBiasTest) {
  int num_knots = 15;

  Eigen::Vector3d bg, ba;
  bg = Eigen::Vector3d::Random() / 100;
  ba = Eigen::Vector3d::Random() / 10;

  basalt::Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  Eigen::aligned_vector<Eigen::Vector3d> accel_data_vec;
  Eigen::aligned_vector<Eigen::Vector3d> gyro_data_vec;
  Eigen::aligned_vector<int64_t> timestamps_vec;

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(1e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - basalt::constants::g);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    accel_data_vec.emplace_back(accel_body + ba);
    gyro_data_vec.emplace_back(rot_vel_body + bg);
    timestamps_vec.emplace_back(t_ns + dt_ns / 2);
  }

  basalt::IntegratedImuMeasurement imu_meas(0, bg, ba);

  for (size_t i = 0; i < timestamps_vec.size(); i++) {
    basalt::ImuData data;
    data.accel = accel_data_vec[i];
    data.gyro = gyro_data_vec[i];
    data.t_ns = timestamps_vec[i];

    imu_meas.integrate(data, Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
  }

  basalt::PoseVelState state0, state1;

  state0.T_w_i = gt_spline.pose(int64_t(0));
  state0.vel_w_i = gt_spline.transVelWorld(int64_t(0));

  state1.T_w_i = gt_spline.pose(imu_meas.get_dt_ns()) *
                 Sophus::se3_expd(Sophus::Vector6d::Random() / 10);
  state1.vel_w_i = gt_spline.transVelWorld(imu_meas.get_dt_ns()) +
                   Sophus::Vector3d::Random() / 10;

  Eigen::Vector3d bg_test = bg + Eigen::Vector3d::Random() / 1000;
  Eigen::Vector3d ba_test = ba + Eigen::Vector3d::Random() / 100;

  basalt::IntegratedImuMeasurement::MatN3 d_res_d_ba, d_res_d_bg;

  basalt::PoseVelState::VecN res =
      imu_meas.residual(state0, basalt::constants::g, state1, bg_test, ba_test,
                        nullptr, nullptr, &d_res_d_bg, &d_res_d_ba);

  {
    basalt::IntegratedImuMeasurement imu_meas1(0, bg_test, ba_test);

    for (size_t i = 0; i < timestamps_vec.size(); i++) {
      basalt::ImuData data;
      data.accel = accel_data_vec[i];
      data.gyro = gyro_data_vec[i];
      data.t_ns = timestamps_vec[i];

      imu_meas1.integrate(data, Eigen::Vector3d::Ones(),
                          Eigen::Vector3d::Ones());
    }

    basalt::PoseVelState::VecN res1 = imu_meas1.residual(
        state0, basalt::constants::g, state1, bg_test, ba_test);

    EXPECT_TRUE(res.isApprox(res1, 1e-4))
        << "res\n"
        << res.transpose() << "\nres1\n"
        << res1.transpose() << "\ndiff\n"
        << (res - res1).transpose() << std::endl;
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_ba", d_res_d_ba,
        [&](const Sophus::Vector3d& x) {
          basalt::IntegratedImuMeasurement imu_meas1(0, bg_test, ba_test + x);

          for (size_t i = 0; i < timestamps_vec.size(); i++) {
            basalt::ImuData data;
            data.accel = accel_data_vec[i];
            data.gyro = gyro_data_vec[i];
            data.t_ns = timestamps_vec[i];

            imu_meas1.integrate(data, Eigen::Vector3d::Ones(),
                                Eigen::Vector3d::Ones());
          }

          return imu_meas1.residual(state0, basalt::constants::g, state1,
                                    bg_test, ba_test + x);
        },
        x0);
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_bg", d_res_d_bg,
        [&](const Sophus::Vector3d& x) {
          basalt::IntegratedImuMeasurement imu_meas1(0, bg_test + x, ba_test);

          for (size_t i = 0; i < timestamps_vec.size(); i++) {
            basalt::ImuData data;
            data.accel = accel_data_vec[i];
            data.gyro = gyro_data_vec[i];
            data.t_ns = timestamps_vec[i];

            imu_meas1.integrate(data, Eigen::Vector3d::Ones(),
                                Eigen::Vector3d::Ones());
          }

          return imu_meas1.residual(state0, basalt::constants::g, state1,
                                    bg_test + x, ba_test);
        },
        x0, 1e-8, 1e-2);
  }
}

TEST(ImuPreintegrationTestCase, CovarianceTest) {
  int num_knots = 15;

  basalt::Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  Eigen::aligned_vector<Eigen::Vector3d> accel_data_vec;
  Eigen::aligned_vector<Eigen::Vector3d> gyro_data_vec;
  Eigen::aligned_vector<int64_t> timestamps_vec;

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(1e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - basalt::constants::g);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    accel_data_vec.emplace_back(accel_body);
    gyro_data_vec.emplace_back(rot_vel_body);
    timestamps_vec.emplace_back(t_ns + dt_ns / 2);
  }

  Eigen::Vector3d accel_cov, gyro_cov;
  accel_cov.setConstant(accel_std_dev * accel_std_dev);
  gyro_cov.setConstant(gyro_std_dev * gyro_std_dev);

  basalt::IntegratedImuMeasurement imu_meas(0, Eigen::Vector3d::Zero(),
                                            Eigen::Vector3d::Zero());

  for (size_t i = 0; i < timestamps_vec.size(); i++) {
    basalt::ImuData data;
    data.accel = accel_data_vec[i];
    data.gyro = gyro_data_vec[i];
    data.t_ns = timestamps_vec[i];

    // std::cerr << "data.accel " << data.accel.transpose() << std::endl;

    // std::cerr << "cov " << i << "\n" << imu_meas.get_cov() << std::endl;
    imu_meas.integrate(data, accel_cov, gyro_cov);
  }

  // std::cerr << "cov\n" << imu_meas.get_cov() << std::endl;

  basalt::PoseVelState delta_state = imu_meas.getDeltaState();

  basalt::IntegratedImuMeasurement::MatNN cov_computed;
  cov_computed.setZero();

  const int num_samples = 1000;
  for (int i = 0; i < num_samples; i++) {
    basalt::IntegratedImuMeasurement imu_meas1(0, Eigen::Vector3d::Zero(),
                                               Eigen::Vector3d::Zero());

    for (size_t i = 0; i < timestamps_vec.size(); i++) {
      basalt::ImuData data;
      data.accel = accel_data_vec[i];
      data.gyro = gyro_data_vec[i];
      data.t_ns = timestamps_vec[i];

      data.accel[0] += accel_noise_dist(gen);
      data.accel[1] += accel_noise_dist(gen);
      data.accel[2] += accel_noise_dist(gen);

      data.gyro[0] += gyro_noise_dist(gen);
      data.gyro[1] += gyro_noise_dist(gen);
      data.gyro[2] += gyro_noise_dist(gen);

      imu_meas1.integrate(data, accel_cov, gyro_cov);
    }

    basalt::PoseVelState delta_state1 = imu_meas1.getDeltaState();

    basalt::PoseVelState::VecN diff = delta_state.diff(delta_state1);

    cov_computed += diff * diff.transpose();
  }

  cov_computed /= num_samples;
  // std::cerr << "cov_computed\n" << cov_computed << std::endl;

  double kl =
      (imu_meas.get_cov_inv() * cov_computed).trace() - 9 +
      std::log(imu_meas.get_cov().determinant() / cov_computed.determinant());

  // std::cerr << "kl " << kl << std::endl;
  EXPECT_LE(kl, 0.08);

  Eigen::VectorXd test_vec(num_samples);
  for (int i = 0; i < num_samples; i++) {
    test_vec[i] = accel_noise_dist(gen);
  }

  double mean = test_vec.mean();
  double var = (test_vec.array() - mean).square().sum() / num_samples;

  // Small test for rangdom generator
  EXPECT_LE(std::abs(std::sqrt(var) - accel_std_dev), 0.03);
}

TEST(ImuPreintegrationTestCase, RandomWalkTest) {
  double dt = 0.005;

  double period = 200;
  double period_dt = period * dt;

  int num_samples = 10000;

  Eigen::VectorXd test_vec(num_samples);
  for (int j = 0; j < num_samples; j++) {
    double test = 0;
    for (int i = 0; i < period; i++) {
      test += gyro_noise_dist(gen) * std::sqrt(dt);
    }
    test_vec[j] = test;
  }

  double mean = test_vec.mean();
  double var = (test_vec.array() - mean).square().sum() / num_samples;
  double std = std::sqrt(var);

  EXPECT_NEAR(gyro_std_dev * std::sqrt(period_dt), std, 1e-4);
  EXPECT_NEAR(gyro_std_dev * gyro_std_dev * period_dt, var, 1e-6);
}
