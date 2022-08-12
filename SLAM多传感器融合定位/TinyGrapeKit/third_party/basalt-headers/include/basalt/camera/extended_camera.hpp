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
@brief Implementation of extended unified camera model
*/

#pragma once

#include <Eigen/Dense>

#include <basalt/utils/sophus_utils.hpp>

namespace basalt {

/// @brief Extended unified camera model
///
/// \image html eucm.png
/// This model has N=6 parameters \f$ \mathbf{i} = \left[f_x, f_y, c_x, c_y,
/// \alpha, \beta \right]^T \f$ with \f$ \alpha \in [0,1], \beta > 0 \f$.
/// See \ref project and \ref unproject functions for more details.
template <typename Scalar>
class ExtendedUnifiedCamera {
 public:
  static constexpr int N = 6;  ///< Number of intrinsic parameters.

  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

  using VecN = Eigen::Matrix<Scalar, N, 1>;

  using Mat24 = Eigen::Matrix<Scalar, 2, 4>;
  using Mat2N = Eigen::Matrix<Scalar, 2, N>;

  using Mat42 = Eigen::Matrix<Scalar, 4, 2>;
  using Mat4N = Eigen::Matrix<Scalar, 4, N>;

  /// @brief Default constructor with zero intrinsics
  ExtendedUnifiedCamera() { param.setZero(); }

  /// @brief Construct camera model with given vector of intrinsics
  ///
  /// @param[in] p vector of intrinsic parameters [fx, fy, cx, cy, alpha, beta]
  explicit ExtendedUnifiedCamera(const VecN& p) { param = p; }

  /// @brief Cast to different scalar type
  template <class Scalar2>
  ExtendedUnifiedCamera<Scalar2> cast() const {
    return ExtendedUnifiedCamera<Scalar2>(param.template cast<Scalar2>());
  }

  /// @brief Camera model name
  ///
  /// @return "eucm"
  static const std::string getName() { return "eucm"; }

  /// @brief Project the point and optionally compute Jacobians
  ///
  /// Projection function is defined as follows:
  /// \f{align}{
  ///  \pi(\mathbf{x}, \mathbf{i}) &=
  ///     \begin{bmatrix}
  ///     f_x{\frac{x}{\alpha d + (1-\alpha) z}}
  ///     \\ f_y{\frac{y}{\alpha d + (1-\alpha) z}}
  ///     \\ \end{bmatrix}
  ///     +
  ///     \begin{bmatrix}
  ///     c_x
  ///     \\ c_y
  ///     \\ \end{bmatrix},
  ///     \\ d &= \sqrt{\beta(x^2 + y^2) + z^2}.
  /// \f}
  /// A set of 3D points that results in valid projection is expressed as
  /// follows: \f{align}{
  /// \Omega &= \{\mathbf{x} \in \mathbb{R}^3 ~|~ z > -wd \},
  /// \\ w &= \begin{cases} \frac{\alpha}{1-\alpha}, & \mbox{if } \alpha \le
  /// 0.5,
  /// \\ \frac{1-\alpha}{\alpha} & \mbox{if } \alpha > 0.5, \end{cases} \f}
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
    const Scalar& alpha = param[4];
    const Scalar& beta = param[5];

    const Scalar& x = p3d[0];
    const Scalar& y = p3d[1];
    const Scalar& z = p3d[2];

    const Scalar r2 = x * x + y * y;
    const Scalar rho2 = beta * r2 + z * z;
    const Scalar rho = sqrt(rho2);

    const Scalar norm = alpha * rho + (Scalar(1) - alpha) * z;

    const Scalar mx = x / norm;
    const Scalar my = y / norm;

    proj = Vec2(fx * mx + cx, fy * my + cy);

    // Check if valid
    const Scalar w = alpha > Scalar(0.5) ? (Scalar(1) - alpha) / alpha
                                         : alpha / (Scalar(1) - alpha);
    if (z <= -w * rho) return false;

    if (d_proj_d_p3d) {
      const Scalar denom = norm * norm * rho;
      const Scalar mid = -(alpha * beta * x * y);
      const Scalar add = norm * rho;
      const Scalar addz = (alpha * z + (Scalar(1) - alpha) * rho);

      (*d_proj_d_p3d)(0, 0) = fx * (add - x * x * alpha * beta);
      (*d_proj_d_p3d)(1, 0) = fy * mid;
      (*d_proj_d_p3d)(0, 1) = fx * mid;
      (*d_proj_d_p3d)(1, 1) = fy * (add - y * y * alpha * beta);
      (*d_proj_d_p3d)(0, 2) = -fx * x * addz;
      (*d_proj_d_p3d)(1, 2) = -fy * y * addz;
      (*d_proj_d_p3d)(0, 3) = Scalar(0);
      (*d_proj_d_p3d)(1, 3) = Scalar(0);

      (*d_proj_d_p3d) /= denom;
    }

    if (d_proj_d_param) {
      const Scalar norm2 = norm * norm;

      (*d_proj_d_param).setZero();
      (*d_proj_d_param)(0, 0) = mx;
      (*d_proj_d_param)(0, 2) = Scalar(1);
      (*d_proj_d_param)(1, 1) = my;
      (*d_proj_d_param)(1, 3) = Scalar(1);

      const Scalar tmp_x = -fx * x / norm2;
      const Scalar tmp_y = -fy * y / norm2;

      const Scalar tmp4 = (rho - z);

      (*d_proj_d_param)(0, 4) = tmp_x * tmp4;
      (*d_proj_d_param)(1, 4) = tmp_y * tmp4;

      const Scalar tmp5 = Scalar(0.5) * alpha * r2 / rho;

      (*d_proj_d_param)(0, 5) = tmp_x * tmp5;
      (*d_proj_d_param)(1, 5) = tmp_y * tmp5;
    }

    return true;
  }

  /// @brief Unproject the point and optionally compute Jacobians
  ///
  /// The unprojection function is computed as follows: \f{align}{
  ///  \pi ^ { -1 }(\mathbf{u}, \mathbf{i}) &=
  ///  \frac{1} {
  ///    \sqrt { m_x ^ 2 + m_y ^ 2 + m_z ^ 2 }
  ///  }
  ///  \begin{bmatrix}  m_x \\  m_y \\  m_z \\  \end{bmatrix},
  ///  \\ m_x &= \frac{u - c_x}{f_x},
  ///  \\ m_y &= \frac{v - c_y}{f_y},
  ///  \\ r^2 &= m_x^2 + m_y^2,
  ///  \\ m_z &= \frac{1 - \beta \alpha^2 r^2}{\alpha \sqrt{1 - (2\alpha - 1)
  ///  \beta r^2} + (1 - \alpha)}. \f}
  ///
  /// The valid range of unprojections is \f{align}{
  ///  \Theta &= \begin{cases}
  ///  \mathbb{R}^2 & \mbox{if } \alpha \le 0.5
  ///  \\ \{ \mathbf{u} \in \mathbb{R}^2 ~|~ r^2 \le \frac{1}{\beta(2\alpha-1)}
  ///  \} & \mbox{if } \alpha > 0.5 \end{cases}
  /// \f}
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

    const Scalar& alpha = param[4];
    const Scalar& beta = param[5];

    const Scalar mx = (proj[0] - cx) / fx;
    const Scalar my = (proj[1] - cy) / fy;

    const Scalar r2 = mx * mx + my * my;
    const Scalar gamma = Scalar(1) - alpha;

    // Check if valid
    if (alpha > Scalar(0.5)) {
      if (r2 >= Scalar(1) / ((alpha - gamma) * beta)) return false;
    }

    const Scalar tmp1 = (Scalar(1) - alpha * alpha * beta * r2);
    const Scalar tmp_sqrt = sqrt(Scalar(1) - (alpha - gamma) * beta * r2);
    const Scalar tmp2 = (alpha * tmp_sqrt + gamma);

    const Scalar k = tmp1 / tmp2;

    const Scalar norm = sqrt(r2 + k * k);

    p3d = Vec4(mx, my, k, 0) / norm;

    if (d_p3d_d_proj || d_p3d_d_param) {
      const Scalar norm2 = norm * norm;

      const Scalar tmp2_2 = tmp2 * tmp2;

      const Scalar d_k_d_r2 =
          Scalar(0.5) * alpha * beta *
          (-Scalar(2) * alpha * tmp2 + tmp1 * (alpha - gamma) / tmp_sqrt) /
          tmp2_2;

      const Scalar d_norm_inv_d_r2 =
          -Scalar(0.5) * (Scalar(1) + Scalar(2) * k * d_k_d_r2) / norm2;

      Vec4 c0, c1;
      c0[0] = (1 + 2 * mx * mx * d_norm_inv_d_r2);
      c0[1] = (2 * my * mx * d_norm_inv_d_r2);
      c0[2] = 2 * mx * (k * d_norm_inv_d_r2 + d_k_d_r2);
      c0[3] = Scalar(0);
      c0 /= fx * norm;

      c1[0] = (2 * my * mx * d_norm_inv_d_r2);
      c1[1] = (1 + 2 * my * my * d_norm_inv_d_r2);
      c1[2] = 2 * my * (k * d_norm_inv_d_r2 + d_k_d_r2);
      c1[3] = Scalar(0);
      c1 /= fy * norm;

      if (d_p3d_d_proj) {
        d_p3d_d_proj->col(0) = c0;
        d_p3d_d_proj->col(1) = c1;
      }

      if (d_p3d_d_param) {
        (*d_p3d_d_param).col(2) = -c0;
        (*d_p3d_d_param).col(3) = -c1;

        (*d_p3d_d_param).col(0) = d_p3d_d_param->col(2) * mx;
        (*d_p3d_d_param).col(1) = d_p3d_d_param->col(3) * my;

        const Scalar d_k_d_alpha =
            (-Scalar(2) * alpha * beta * r2 * tmp2 -
             (tmp_sqrt - alpha * beta * r2 / tmp_sqrt - Scalar(1)) * tmp1) /
            tmp2_2;

        const Scalar d_k_d_beta =
            alpha * r2 *
            (Scalar(0.5) * tmp1 * (alpha - gamma) / tmp_sqrt - alpha * tmp2) /
            tmp2_2;

        const Scalar d_norm_inv_d_k = -k / norm2;

        (*d_p3d_d_param)(0, 4) = mx * d_norm_inv_d_k * d_k_d_alpha;
        (*d_p3d_d_param)(1, 4) = my * d_norm_inv_d_k * d_k_d_alpha;
        (*d_p3d_d_param)(2, 4) = (k * d_norm_inv_d_k + 1) * d_k_d_alpha;
        (*d_p3d_d_param)(3, 4) = Scalar(0);
        d_p3d_d_param->col(4) /= norm;

        (*d_p3d_d_param)(0, 5) = mx * d_norm_inv_d_k * d_k_d_beta;
        (*d_p3d_d_param)(1, 5) = my * d_norm_inv_d_k * d_k_d_beta;
        (*d_p3d_d_param)(2, 5) = (k * d_norm_inv_d_k + 1) * d_k_d_beta;
        (*d_p3d_d_param)(3, 5) = Scalar(0);
        d_p3d_d_param->col(5) /= norm;
      }
    }

    return true;
  }

  /// @brief Set parameters from initialization
  ///
  /// Initializes the camera model to  \f$ \left[f_x, f_y, c_x, c_y, 0.5, 1
  /// \right]^T \f$
  ///
  /// @param[in] init vector [fx, fy, cx, cy]
  inline void setFromInit(const Vec4& init) {
    param[0] = init[0];
    param[1] = init[1];
    param[2] = init[2];
    param[3] = init[3];
    param[4] = 0.5;
    param[5] = 1;
  }

  /// @brief Increment intrinsic parameters by inc and clamp the values to the
  /// valid range
  ///
  /// @param[in] inc increment vector
  void operator+=(const VecN& inc) {
    param += inc;

    // alpha in [0, 1], beta > 0
    param[4] = std::clamp(param[4], Scalar(0), Scalar(1));
    if (param[5] < Sophus::Constants<Scalar>::epsilonSqrt())
      param[5] = Sophus::Constants<Scalar>::epsilonSqrt();
  }

  /// @brief Returns a const reference to the intrinsic parameters vector
  ///
  /// The order is following: \f$ \left[f_x, f_y, c_x, c_y, \alpha, \beta
  /// \right]^T \f$
  /// @return const reference to the intrinsic parameters vector
  const VecN& getParam() const { return param; }

  /// @brief Projections used for unit-tests
  static Eigen::aligned_vector<ExtendedUnifiedCamera> getTestProjections() {
    Eigen::aligned_vector<ExtendedUnifiedCamera> res;

    VecN vec1;

    // Euroc
    vec1 << 460.76484651566468, 459.4051018049483, 365.8937161309615,
        249.33499869752445, 0.5903365915227143, 1.127468196965374;
    res.emplace_back(vec1);

    // TUM VI 512
    vec1 << 191.14799816648748, 191.13150946585135, 254.95857715233118,
        256.8815466235898, 0.6291060871161842, 1.0418067403139693;
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
