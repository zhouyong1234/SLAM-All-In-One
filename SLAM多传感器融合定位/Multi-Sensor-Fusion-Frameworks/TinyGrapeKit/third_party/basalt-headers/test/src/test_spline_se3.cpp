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

#include <basalt/spline/se3_spline.h>

#include <iostream>

#include "gtest/gtest.h"
#include "test_utils.h"

template <int N>
void test_gyro_res(const basalt::Se3Spline<N> &s, int64_t t_ns) {
  typename basalt::Se3Spline<N>::SO3JacobianStruct Ja;
  Eigen::Matrix<double, 3, 12> J_bias;

  basalt::CalibGyroBias<double> bias;

  bias.setRandom();

  // bias << 0.01, -0.02, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  Eigen::Vector3d measurement = s.rotVelBody(t_ns);

  s.gyroResidual(t_ns, measurement, bias, &Ja, &J_bias);

  for (size_t i = 0; i < s.numKnots(); i++) {
    Sophus::Vector3d x0;
    x0.setZero();

    std::stringstream ss;
    ss << "Spline order " << N << " d_gyro_res_d_knot" << i << " time " << t_ns;

    Sophus::Matrix3d J;

    if (i >= Ja.start_idx && i < Ja.start_idx + N) {
      J = Ja.d_val_d_knot[i - Ja.start_idx];
    } else {
      J.setZero();
    }

    test_jacobian(
        ss.str(), J,
        [&](const Sophus::Vector3d &x) {
          basalt::Se3Spline<N> s1 = s;
          s1.getKnotSO3(i) = Sophus::SO3d::exp(x) * s.getKnotSO3(i);

          return s1.gyroResidual(t_ns, measurement, bias);
        },
        x0);
  }

  {
    Eigen::Matrix<double, 12, 1> x0;
    x0.setZero();

    std::stringstream ss;
    ss << "Spline order " << N << " d_gyro_res_d_bias";

    test_jacobian(
        ss.str(), J_bias,
        [&](const Eigen::Matrix<double, 12, 1> &x) {
          auto b1 = bias;
          b1 += x;
          return s.gyroResidual(t_ns, measurement, b1);
        },
        x0);
  }
}

template <int N>
void test_accel_res(const basalt::Se3Spline<N> &s, int64_t t_ns) {
  typename basalt::Se3Spline<N>::AccelPosSO3JacobianStruct Ja;
  Eigen::Matrix3d J_g;

  Eigen::Matrix<double, 3, 9> J_bias;

  basalt::CalibAccelBias<double> bias;
  bias.setRandom();
  // bias << -0.4, 0.5, -0.6, 0, 0, 0, 0, 0, 0;

  Eigen::Vector3d g(0, 0, -9.81);
  Eigen::Vector3d measurement = s.transAccelWorld(t_ns) + g;

  s.accelResidual(t_ns, measurement, bias, g, &Ja, &J_bias, &J_g);

  for (size_t i = 0; i < s.numKnots(); i++) {
    Sophus::Vector6d x0;
    x0.setZero();

    std::stringstream ss;
    ss << "Spline order " << N << " d_accel_res_d_knot" << i;

    typename basalt::Se3Spline<N>::Mat36 J;

    if (i >= Ja.start_idx && i < Ja.start_idx + N) {
      J = Ja.d_val_d_knot[i - Ja.start_idx];
    } else {
      J.setZero();
    }

    test_jacobian(
        ss.str(), J,
        [&](const Sophus::Vector6d &x) {
          basalt::Se3Spline<N> s1 = s;
          s1.applyInc(i, x);

          return s1.accelResidual(t_ns, measurement, bias, g);
        },
        x0);
  }

  {
    Eigen::Matrix<double, 9, 1> x0;
    x0.setZero();

    std::stringstream ss;
    ss << "Spline order " << N << " d_accel_res_d_bias";

    test_jacobian(
        ss.str(), J_bias,
        [&](const Eigen::Matrix<double, 9, 1> &x) {
          auto b1 = bias;
          b1 += x;
          return s.accelResidual(t_ns, measurement, b1, g);
        },
        x0);
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();

    std::stringstream ss;
    ss << "Spline order " << N << " d_accel_res_d_g";

    test_jacobian(
        ss.str(), J_g,
        [&](const Sophus::Vector3d &x) {
          return s.accelResidual(t_ns, measurement, bias, g + x);
        },
        x0);
  }
}

template <int N>
void test_orientation_res(const basalt::Se3Spline<N> &s, int64_t t_ns) {
  typename basalt::Se3Spline<N>::SO3JacobianStruct Ja;

  Sophus::SO3d measurement =
      s.pose(t_ns).so3();  // * Sophus::expd(Sophus::Vector6d::Random() / 10);

  s.orientationResidual(t_ns, measurement, &Ja);

  for (size_t i = 0; i < s.numKnots(); i++) {
    std::stringstream ss;
    ss << "Spline order " << N << " d_rot_res_d_knot" << i;

    Sophus::Matrix3d J;

    if (i >= Ja.start_idx && i < Ja.start_idx + N) {
      J = Ja.d_val_d_knot[i - Ja.start_idx];
    } else {
      J.setZero();
    }

    Sophus::Vector3d x0;
    x0.setZero();

    test_jacobian(
        ss.str(), J,
        [&](const Sophus::Vector3d &x_rot) {
          Sophus::Vector6d x;
          x.setZero();
          x.tail<3>() = x_rot;

          basalt::Se3Spline<N> s1 = s;
          s1.applyInc(i, x);

          return s1.orientationResidual(t_ns, measurement);
        },
        x0);
  }
}

template <int N>
void test_position_res(const basalt::Se3Spline<N> &s, int64_t t_ns) {
  typename basalt::Se3Spline<N>::PosJacobianStruct Ja;

  Eigen::Vector3d measurement =
      s.pose(t_ns)
          .translation();  // * Sophus::expd(Sophus::Vector6d::Random() / 10);

  s.positionResidual(t_ns, measurement, &Ja);

  for (size_t i = 0; i < s.numKnots(); i++) {
    std::stringstream ss;
    ss << "Spline order " << N << " d_pos_res_d_knot" << i;

    Sophus::Matrix3d J;
    J.setZero();

    if (i >= Ja.start_idx && i < Ja.start_idx + N) {
      J.diagonal().setConstant(Ja.d_val_d_knot[i - Ja.start_idx]);
    }

    Sophus::Vector3d x0;
    x0.setZero();

    test_jacobian(
        ss.str(), J,
        [&](const Sophus::Vector3d &x_rot) {
          Sophus::Vector6d x;
          x.setZero();
          x.head<3>() = x_rot;

          basalt::Se3Spline<N> s1 = s;
          s1.applyInc(i, x);

          return s1.positionResidual(t_ns, measurement);
        },
        x0);
  }
}

template <int N>
void test_pose(const basalt::Se3Spline<N> &s, int64_t t_ns) {
  typename basalt::Se3Spline<N>::PosePosSO3JacobianStruct Ja;

  Sophus::SE3d res = s.pose(t_ns, &Ja);

  Sophus::Vector6d x0;
  x0.setZero();

  for (size_t i = 0; i < s.numKnots(); i++) {
    std::stringstream ss;
    ss << "Spline order " << N << " d_pose_d_knot" << i;

    typename basalt::Se3Spline<N>::Mat6 J;

    if (i >= Ja.start_idx && i < Ja.start_idx + N) {
      J = Ja.d_val_d_knot[i - Ja.start_idx];
    } else {
      J.setZero();
    }

    test_jacobian(
        ss.str(), J,
        [&](const Sophus::Vector6d &x) {
          basalt::Se3Spline<N> s1 = s;
          s1.applyInc(i, x);

          return Sophus::se3_logd(res.inverse() * s1.pose(t_ns));
        },
        x0);
  }

  {
    Eigen::Matrix<double, 1, 1> x0;
    x0[0] = 0;

    typename basalt::Se3Spline<N>::Vec6 J;

    //    J.template head<3>() = res.so3().inverse() * s.transVelWorld(t_ns);
    //    J.template tail<3>() = s.rotVelBody(t_ns);

    s.d_pose_d_t(t_ns, J);

    test_jacobian(
        "J_pose_time", J,
        [&](const Eigen::Matrix<double, 1, 1> &x) {
          int64_t t_ns_new = t_ns;
          t_ns_new += x[0] * 1e9;

          return Sophus::se3_logd(res.inverse() * s.pose(t_ns_new));
        },
        x0);
  }
}

TEST(SplineSE3, GyroResidualTest) {
  static const int N = 5;

  const int num_knots = 3 * N;
  basalt::Se3Spline<N> s(int64_t(2e9));
  s.genRandomTrajectory(num_knots);

  for (int64_t t_ns = 0; t_ns < s.maxTimeNs(); t_ns += 1e8) {
    test_gyro_res<N>(s, t_ns);
  }
}

TEST(SplineSE3, AccelResidualTest) {
  static const int N = 5;

  const int num_knots = 3 * N;
  basalt::Se3Spline<N> s(int64_t(2e9));
  s.genRandomTrajectory(num_knots);

  for (int64_t t_ns = 0; t_ns < s.maxTimeNs(); t_ns += 1e8) {
    test_accel_res<N>(s, t_ns);
  }
}

TEST(SplineSE3, PositionResidualTest) {
  static const int N = 5;

  const int num_knots = 3 * N;
  basalt::Se3Spline<N> s(int64_t(2e9));
  s.genRandomTrajectory(num_knots);

  for (int64_t t_ns = 0; t_ns < s.maxTimeNs(); t_ns += 1e8) {
    test_position_res<N>(s, t_ns);
  }
}

TEST(SplineSE3, OrientationResidualTest) {
  static const int N = 5;

  const int num_knots = 3 * N;
  basalt::Se3Spline<N> s(int64_t(2e9));
  s.genRandomTrajectory(num_knots);

  for (int64_t t_ns = 0; t_ns < s.maxTimeNs(); t_ns += 1e8) {
    test_orientation_res<N>(s, t_ns);
  }
}

TEST(SplineSE3, PoseTest) {
  static const int N = 5;

  const int num_knots = 3 * N;
  basalt::Se3Spline<N> s(int64_t(2e9));
  s.genRandomTrajectory(num_knots);

  int64_t offset = 100;

  for (int64_t t_ns = offset; t_ns < s.maxTimeNs() - offset; t_ns += 1e8) {
    test_pose<N>(s, t_ns);
  }
}
