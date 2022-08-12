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
@brief Implementation of unified camera model
*/

#pragma once

#include <basalt/utils/sophus_utils.hpp>

namespace basalt {

/// @brief Camera model used in the paper "Bundle Adjustment in the Large".
///
/// See https://grail.cs.washington.edu/projects/bal/ for details.
/// This model has N=3 parameters \f$ \mathbf{i} = \left[f, k_1, k_2
/// \right]^T \f$.
///
/// Unlike the original formulation we assume that the POSITIVE z-axis
/// points in camera direction and thus don't include the "minus" in the
/// perspective projection. You need to consider this when loading BAL data.
///
/// Specifically, for the camera frame we assume the positive z axis pointing
/// forward in view direction and in the image, y is poiting down, x to the
/// right. In the original BAL formulation, the camera points in negative z
/// axis, y is up in the image. Thus when loading the data, we invert the y and
/// z camera axes (y also in the image) in the perspective projection, we don't
/// have the "minus" like in the original Snavely model.
///
/// A 3D point P in camera coordinates is mapped to pixel coordinates p':
/// p  = [P / P.z]_xy    (perspective division)
/// p' =  f * r(p) * p   (conversion to pixel coordinates)
/// r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.
///
/// See \ref project and \ref unproject functions for more details.
template <typename Scalar = double>
class BalCamera {
 public:
  static constexpr int N = 3;  ///< Number of intrinsic parameters.

  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

  using VecN = Eigen::Matrix<Scalar, N, 1>;

  using Mat2 = Eigen::Matrix<Scalar, 2, 2>;
  using Mat24 = Eigen::Matrix<Scalar, 2, 4>;
  using Mat2N = Eigen::Matrix<Scalar, 2, N>;

  using Mat42 = Eigen::Matrix<Scalar, 4, 2>;
  using Mat4N = Eigen::Matrix<Scalar, 4, N>;

  /// @brief Default constructor with zero intrinsics
  BalCamera() { param.setZero(); }

  /// @brief Construct camera model with given vector of intrinsics
  ///
  /// @param[in] p vector of intrinsic parameters [f, k1, k2]
  explicit BalCamera(const VecN& p) { param = p; }

  /// @brief Cast to different scalar type
  template <class Scalar2>
  BalCamera<Scalar2> cast() const {
    return BalCamera<Scalar2>(param.template cast<Scalar2>());
  }

  /// @brief Camera model name
  ///
  /// @return "bal"
  static const std::string getName() { return "bal"; }

  /// @brief Project the point and optionally compute Jacobians
  ///
  /// @param[in] p3d point to project
  /// @param[out] proj result of projection
  /// @param[out] d_proj_d_p3d if not nullptr computed Jacobian of projection
  /// with respect to p3d
  /// @param[out] d_proj_d_param point if not nullptr computed Jacobian of
  /// projection with respect to intrinsic parameters
  /// @return if projection is valid
  inline bool project(const Vec4& p3d, Vec2& proj,
                      Mat24* d_proj_d_p3d = nullptr,
                      Mat2N* d_proj_d_param = nullptr) const {
    const Scalar& f = param[0];
    const Scalar& k1 = param[1];
    const Scalar& k2 = param[2];

    const Scalar& x = p3d[0];
    const Scalar& y = p3d[1];
    const Scalar& z = p3d[2];

    const Scalar mx = x / z;
    const Scalar my = y / z;

    const Scalar mx2 = mx * mx;
    const Scalar my2 = my * my;

    const Scalar r2 = mx2 + my2;
    const Scalar r4 = r2 * r2;

    const Scalar rp = Scalar(1) + k1 * r2 + k2 * r4;

    proj = Vec2(f * mx * rp, f * my * rp);

    if (z < Sophus::Constants<Scalar>::epsilonSqrt()) return false;

    if (d_proj_d_p3d) {
      d_proj_d_p3d->setZero();

      const Scalar tmp = k1 + k2 * Scalar(2) * r2;

      (*d_proj_d_p3d)(0, 0) = f * (rp + Scalar(2) * mx2 * tmp) / z;
      (*d_proj_d_p3d)(1, 1) = f * (rp + Scalar(2) * my2 * tmp) / z;

      (*d_proj_d_p3d)(1, 0) = (*d_proj_d_p3d)(0, 1) =
          f * my * mx * Scalar(2) * tmp / z;

      (*d_proj_d_p3d)(0, 2) = -f * mx * (rp + Scalar(2) * tmp * r2) / z;
      (*d_proj_d_p3d)(1, 2) = -f * my * (rp + Scalar(2) * tmp * r2) / z;
    }

    if (d_proj_d_param) {
      (*d_proj_d_param).setZero();
      (*d_proj_d_param)(0, 0) = mx * rp;
      (*d_proj_d_param)(1, 0) = my * rp;
      (*d_proj_d_param)(0, 1) = f * mx * r2;
      (*d_proj_d_param)(1, 1) = f * my * r2;
      (*d_proj_d_param)(0, 2) = f * mx * r4;
      (*d_proj_d_param)(1, 2) = f * my * r4;
    }

    return true;
  }

  /// @brief Unproject the point and optionally compute Jacobians
  ///
  ///
  /// @param[in] proj point to unproject
  /// @param[out] p3d result of unprojection
  /// @param[out] d_p3d_d_proj if not nullptr computed Jacobian of unprojection
  /// with respect to proj
  /// @param[out] d_p3d_d_param point if not nullptr computed Jacobian of
  /// unprojection with respect to intrinsic parameters
  /// @return if unprojection is valid
  inline bool unproject(const Vec2& proj, Vec4& p3d,
                        Mat42* d_p3d_d_proj = nullptr,
                        Mat4N* d_p3d_d_param = nullptr) const {
    const Scalar& f = param[0];
    const Scalar& k1 = param[1];
    const Scalar& k2 = param[2];

    const Scalar& u = proj[0];
    const Scalar& v = proj[1];

    const Vec2 pp(u / f, v / f);

    Vec2 p = pp;

    for (int i = 0; i < 3; i++) {
      const Scalar r2 = p.squaredNorm();

      const Scalar rp = (Scalar(1) + k1 * r2 + k2 * r2 * r2);
      const Vec2 pp_computed = p * rp;

      const Scalar tmp = k1 + k2 * Scalar(2) * r2;

      Mat2 J;
      J(0, 0) = (rp + Scalar(2) * p[0] * p[0] * tmp);
      J(1, 1) = (rp + Scalar(2) * p[1] * p[1] * tmp);
      J(1, 0) = J(0, 1) = p[0] * p[1] * Scalar(2) * tmp;

      const Vec2 dp =
          (J.transpose() * J).inverse() * J.transpose() * (pp_computed - pp);

      p -= dp;
    }

    p3d[0] = p[0];
    p3d[1] = p[1];
    p3d[2] = 1;
    p3d[3] = Scalar(0);

    p3d.normalize();

    BASALT_ASSERT_STREAM(d_p3d_d_proj == nullptr && d_p3d_d_param == nullptr,
                         "Jacobians for unprojection are not implemented");

    return true;
  }

  /// @brief Set parameters from initialization
  ///
  /// Initializes the camera model to  \f$ \left[f_x, 0, 0 \right]^T \f$
  ///
  /// @param[in] init vector [fx, fy, cx, cy]
  inline void setFromInit(const Vec4& init) {
    param[0] = init[0];
    param[1] = 0;
    param[2] = 0;
  }

  /// @brief Increment intrinsic parameters by inc and clamp the values to the
  /// valid range
  ///
  /// @param[in] inc increment vector
  void operator+=(const VecN& inc) { param += inc; }

  /// @brief Returns a const reference to the intrinsic parameters vector
  ///
  /// The order is following: \f$ \left[f, k1, k2 \right]^T \f$
  /// @return const reference to the intrinsic parameters vector
  const VecN& getParam() const { return param; }

  /// @brief Projections used for unit-tests
  static Eigen::aligned_vector<BalCamera> getTestProjections() {
    Eigen::aligned_vector<BalCamera> res;

    VecN vec1;

    vec1 << 399.752, -3.78048e-05, 5.37738e-07;
    res.emplace_back(vec1);

    return res;
  }

  /// @brief Resolutions used for unit-tests
  static Eigen::aligned_vector<Eigen::Vector2i> getTestResolutions() {
    Eigen::aligned_vector<Eigen::Vector2i> res;

    res.emplace_back(640, 480);

    return res;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  VecN param;
};

}  // namespace basalt
