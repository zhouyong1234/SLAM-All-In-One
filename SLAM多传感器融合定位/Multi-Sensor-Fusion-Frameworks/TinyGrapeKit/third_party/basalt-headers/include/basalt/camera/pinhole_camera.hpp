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
@brief Implementation of pinhole camera model
*/

#pragma once

#include <basalt/utils/sophus_utils.hpp>

namespace basalt {

/// @brief Pinhole camera model
///
/// This model has N=4 parameters \f$ \mathbf{i} = \left[f_x, f_y, c_x, c_y
/// \right]^T \f$ with. See \ref
/// project and \ref unproject functions for more details.
template <typename Scalar = double>
class PinholeCamera {
 public:
  static constexpr int N = 4;  ///< Number of intrinsic parameters.

  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

  using VecN = Eigen::Matrix<Scalar, N, 1>;

  using Mat24 = Eigen::Matrix<Scalar, 2, 4>;
  using Mat2N = Eigen::Matrix<Scalar, 2, N>;

  using Mat42 = Eigen::Matrix<Scalar, 4, 2>;
  using Mat4N = Eigen::Matrix<Scalar, 4, N>;

  /// @brief Default constructor with zero intrinsics
  PinholeCamera() { param.setZero(); }

  /// @brief Construct camera model with given vector of intrinsics
  ///
  /// @param[in] p vector of intrinsic parameters [fx, fy, cx, cy]
  explicit PinholeCamera(const VecN& p) { param = p; }

  /// @brief Cast to different scalar type
  template <class Scalar2>
  PinholeCamera<Scalar2> cast() const {
    return PinholeCamera<Scalar2>(param.template cast<Scalar2>());
  }

  /// @brief Camera model name
  ///
  /// @return "pinhole"
  static const std::string getName() { return "pinhole"; }

  /// @brief Project the point and optionally compute Jacobians
  ///
  /// Projection function is defined as follows:
  /// \f{align}{
  ///    \pi(\mathbf{x}, \mathbf{i}) &=
  ///    \begin{bmatrix}
  ///    f_x{\frac{x}{z}}
  ///    \\ f_y{\frac{y}{z}}
  ///    \\ \end{bmatrix}
  ///    +
  ///    \begin{bmatrix}
  ///    c_x
  ///    \\ c_y
  ///    \\ \end{bmatrix}.
  /// \f}
  /// A set of 3D points that results in valid projection is expressed as
  /// follows: \f{align}{
  ///    \Omega &= \{\mathbf{x} \in \mathbb{R}^3 ~|~ z > 0 \}
  /// \f}
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
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];

    const Scalar& x = p3d[0];
    const Scalar& y = p3d[1];
    const Scalar& z = p3d[2];

    proj[0] = fx * x / z + cx;
    proj[1] = fy * y / z + cy;

    if (z < Sophus::Constants<Scalar>::epsilonSqrt()) return false;

    if (d_proj_d_p3d) {
      d_proj_d_p3d->setZero();
      const Scalar z2 = z * z;

      (*d_proj_d_p3d)(0, 0) = fx / z;
      (*d_proj_d_p3d)(0, 2) = -fx * x / z2;

      (*d_proj_d_p3d)(1, 1) = fy / z;
      (*d_proj_d_p3d)(1, 2) = -fy * y / z2;
    }

    if (d_proj_d_param) {
      d_proj_d_param->setZero();
      (*d_proj_d_param)(0, 0) = x / z;
      (*d_proj_d_param)(0, 2) = Scalar(1);
      (*d_proj_d_param)(1, 1) = y / z;
      (*d_proj_d_param)(1, 3) = Scalar(1);
    }

    return true;
  }

  /// @brief Unproject the point and optionally compute Jacobians
  ///
  /// The unprojection function is computed as follows: \f{align}{
  ///    \pi^{-1}(\mathbf{u}, \mathbf{i}) &=
  ///    \frac{1}{m_x^2 + m_y^2 + 1}
  ///    \begin{bmatrix}
  ///    m_x \\ m_y \\ 1
  ///    \\ \end{bmatrix}
  ///    \\ m_x &= \frac{u - c_x}{f_x},
  ///    \\ m_y &= \frac{v - c_y}{f_y}.
  /// \f}
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
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];

    const Scalar mx = (proj[0] - cx) / fx;
    const Scalar my = (proj[1] - cy) / fy;

    const Scalar r2 = mx * mx + my * my;

    const Scalar norm = sqrt(Scalar(1) + r2);
    const Scalar norm_inv = Scalar(1) / norm;

    p3d[0] = mx * norm_inv;
    p3d[1] = my * norm_inv;
    p3d[2] = norm_inv;
    p3d[3] = Scalar(0);

    if (d_p3d_d_proj || d_p3d_d_param) {
      const Scalar d_norm_inv_d_r2 =
          -Scalar(0.5) * norm_inv * norm_inv * norm_inv;

      Vec4 c0, c1;
      c0(0) = (norm_inv + 2 * mx * mx * d_norm_inv_d_r2) / fx;
      c0(1) = (2 * my * mx * d_norm_inv_d_r2) / fx;
      c0(2) = 2 * mx * d_norm_inv_d_r2 / fx;
      c0(3) = Scalar(0);

      c1(0) = (2 * my * mx * d_norm_inv_d_r2) / fy;
      c1(1) = (norm_inv + 2 * my * my * d_norm_inv_d_r2) / fy;
      c1(2) = 2 * my * d_norm_inv_d_r2 / fy;
      c1(3) = Scalar(0);

      if (d_p3d_d_proj) {
        d_p3d_d_proj->col(0) = c0;
        d_p3d_d_proj->col(1) = c1;
      }

      if (d_p3d_d_param) {
        d_p3d_d_param->col(2) = -c0;
        d_p3d_d_param->col(3) = -c1;

        d_p3d_d_param->col(0) = -c0 * mx;
        d_p3d_d_param->col(1) = -c1 * my;
      }
    }

    return true;
  }

  /// @brief Set parameters from initialization
  ///
  /// Initializes the camera model to  \f$ \left[f_x, f_y, c_x, c_y, \right]^T
  /// \f$
  ///
  /// @param[in] init vector [fx, fy, cx, cy]
  inline void setFromInit(const Vec4& init) {
    param[0] = init[0];
    param[1] = init[1];
    param[2] = init[2];
    param[3] = init[3];
  }

  /// @brief Increment intrinsic parameters by inc
  ///
  /// @param[in] inc increment vector
  void operator+=(const VecN& inc) { param += inc; }

  /// @brief Returns a const reference to the intrinsic parameters vector
  ///
  /// The order is following: \f$ \left[f_x, f_y, c_x, c_y, \right]^T \f$
  /// @return const reference to the intrinsic parameters vector
  const VecN& getParam() const { return param; }

  /// @brief Projections used for unit-tests
  static Eigen::aligned_vector<PinholeCamera> getTestProjections() {
    Eigen::aligned_vector<PinholeCamera> res;

    VecN vec1;

    // Euroc
    vec1 << 460.76484651566468, 459.4051018049483, 365.8937161309615,
        249.33499869752445;
    res.emplace_back(vec1);

    // TUM VI 512
    vec1 << 191.14799816648748, 191.13150946585135, 254.95857715233118,
        256.8815466235898;
    res.emplace_back(vec1);

    return res;
  }

  /// @brief Resolutions used for unit-tests
  static Eigen::aligned_vector<Eigen::Vector2i> getTestResolutions() {
    Eigen::aligned_vector<Eigen::Vector2i> res;

    res.emplace_back(752, 480);
    res.emplace_back(512, 512);

    return res;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  VecN param;
};

}  // namespace basalt
