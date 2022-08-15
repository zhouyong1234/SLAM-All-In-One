/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_FEATURE_H
#define MSCKF_VIO_FEATURE_H

#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "math_utils.hpp"
#include "imu_state.h"
#include "cam_state.h"

namespace msckf_vio {

/*
 * @brief Feature Salient part of an image. Please refer
 *    to the Appendix of "A Multi-State Constraint Kalman
 *    Filter for Vision-aided Inertial Navigation" for how
 *    the 3d position of a feature is initialized.
 */
struct Feature {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef long long int FeatureIDType;

  /*
   * @brief OptimizationConfig Configuration parameters
   *    for 3d feature position optimization.
   */
  struct OptimizationConfig {
    double translation_threshold;
    double huber_epsilon;
    double estimation_precision;
    double initial_damping;
    int outer_loop_max_iteration;
    int inner_loop_max_iteration;

    OptimizationConfig():
      translation_threshold(0.2),
      huber_epsilon(0.01),
      estimation_precision(5e-7),
      initial_damping(1e-3),
      outer_loop_max_iteration(10),
      inner_loop_max_iteration(10) {
      return;
    }
  };

  // Constructors for the struct.
  Feature(): id(0), position(Eigen::Vector3d::Zero()),
    is_initialized(false) {}

  Feature(const FeatureIDType& new_id): id(new_id),
    position(Eigen::Vector3d::Zero()),
    is_initialized(false) {}

  /*
   * @brief cost Compute the cost of the camera observations
   * @param T_c0_c1 A rigid body transformation takes
   *    a vector in c0 frame to ci frame.
   * @param x The current estimation.
   * @param z The ith measurement of the feature j in ci frame.
   * @return e The cost of this observation.
   */
  inline void cost(const Eigen::Isometry3d& T_c0_ci,
      const Eigen::Vector3d& x, const Eigen::Vector2d& z,
      double& e) const;

  /*
   * @brief jacobian Compute the Jacobian of the camera observation
   * @param T_c0_c1 A rigid body transformation takes
   *    a vector in c0 frame to ci frame.
   * @param x The current estimation.
   * @param z The actual measurement of the feature in ci frame.
   * @return J The computed Jacobian.
   * @return r The computed residual.
   * @return w Weight induced by huber kernel.
   */
  inline void jacobian(const Eigen::Isometry3d& T_c0_ci,
      const Eigen::Vector3d& x, const Eigen::Vector2d& z,
      Eigen::Matrix<double, 2, 3>& J, Eigen::Vector2d& r,
      double& w) const;

  /*
   * @brief generateInitialGuess Compute the initial guess of
   *    the feature's 3d position using only two views.
   * @param T_c1_c2: A rigid body transformation taking
   *    a vector from c2 frame to c1 frame.
   * @param z1: feature observation in c1 frame.
   * @param z2: feature observation in c2 frame.
   * @return p: Computed feature position in c1 frame.
   */
  inline void generateInitialGuess(
      const Eigen::Isometry3d& T_c1_c2, const Eigen::Vector2d& z1,
      const Eigen::Vector2d& z2, Eigen::Vector3d& p) const;

  /*
   * @brief checkMotion Check the input camera poses to ensure
   *    there is enough translation to triangulate the feature
   *    positon.
   * @param cam_states : input camera poses.
   * @return True if the translation between the input camera
   *    poses is sufficient.
   */
  inline bool checkMotion(
      const CamStateServer& cam_states) const;

  /*
   * @brief InitializePosition Intialize the feature position
   *    based on all current available measurements.
   * @param cam_states: A map containing the camera poses with its
   *    ID as the associated key value.
   * @return The computed 3d position is used to set the position
   *    member variable. Note the resulted position is in world
   *    frame.
   * @return True if the estimated 3d position of the feature
   *    is valid.
   */
  inline bool initializePosition(
      const CamStateServer& cam_states);


  // An unique identifier for the feature.
  // In case of long time running, the variable
  // type of id is set to FeatureIDType in order
  // to avoid duplication.
  FeatureIDType id;

  // id for next feature
  static FeatureIDType next_id;

  // Store the observations of the features in the
  // state_id(key)-image_coordinates(value) manner.
  std::map<StateIDType, Eigen::Vector2d, std::less<StateIDType>,
    Eigen::aligned_allocator<
      std::pair<const StateIDType, Eigen::Vector2d> > > observations;

  // 3d postion of the feature in the world frame.
  Eigen::Vector3d position;

  // A indicator to show if the 3d postion of the feature
  // has been initialized or not.
  bool is_initialized;

  // Noise for a normalized feature measurement.
  static double observation_noise;

  // Optimization configuration for solving the 3d position.
  static OptimizationConfig optimization_config;

};

typedef Feature::FeatureIDType FeatureIDType;
typedef std::map<FeatureIDType, Feature, std::less<int>,
        Eigen::aligned_allocator<
        std::pair<const FeatureIDType, Feature> > > MapServer;

// 利用feature在c0帧的逆深度参数模型，根据c0帧和ci帧间的相对位姿、计算c1帧下feature的重投影误差，并以误差的模平方为返回值。具体参考Eq.(37)的参考文献
void Feature::cost(const Eigen::Isometry3d& T_c0_ci,
    const Eigen::Vector3d& x, const Eigen::Vector2d& z,
    double& e) const {
  // Compute hi1, hi2, and hi3 as Equation (37).    // QXC：Eq.(37)来自"A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation"
  const double& alpha = x(0);
  const double& beta = x(1);
  const double& rho = x(2);

  Eigen::Vector3d h = T_c0_ci.linear()*
    Eigen::Vector3d(alpha, beta, 1.0) + rho*T_c0_ci.translation();
  double& h1 = h(0);
  double& h2 = h(1);
  double& h3 = h(2);

  // Predict the feature observation in ci frame.
  Eigen::Vector2d z_hat(h1/h3, h2/h3);

  // Compute the residual.
  e = (z_hat-z).squaredNorm();
  return;
}

// 计算“基于c0帧下feature逆深度参数模型的重投影误差”关于逆深度参数向量[alpha; beta; rho]的Jacobian矩阵。Jacobian的推导以注释形式给出
void Feature::jacobian(const Eigen::Isometry3d& T_c0_ci,
    const Eigen::Vector3d& x, const Eigen::Vector2d& z,
    Eigen::Matrix<double, 2, 3>& J, Eigen::Vector2d& r,
    double& w) const {

  // Compute hi1, hi2, and hi3 as Equation (37).    // QXC：Eq.(37)来自"A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation"
  const double& alpha = x(0);
  const double& beta = x(1);
  const double& rho = x(2);

  Eigen::Vector3d h = T_c0_ci.linear()*
    Eigen::Vector3d(alpha, beta, 1.0) + rho*T_c0_ci.translation();
  double& h1 = h(0);
  double& h2 = h(1);
  double& h3 = h(2);

  // Compute the Jacobian.
  Eigen::Matrix3d W;
  W.leftCols<2>() = T_c0_ci.linear().leftCols<2>();
  W.rightCols<1>() = T_c0_ci.translation();

  J.row(0) = 1/h3*W.row(0) - h1/(h3*h3)*W.row(2);
  J.row(1) = 1/h3*W.row(1) - h2/(h3*h3)*W.row(2);

  /* QXC：给出上述Jacobian的推导如下：
   *    要求的是Eq.(37)式左边的z关于列向量[alpha; beta; rho]的Jacobian（简写作[al; be; rh]）
   *    设T_c0_ci由R和t组成，其中：
   *        R = [r11, r12, r13;
   *             r21, r22, r23;
   *             r31, r32, r33]
   *        t = [t1; t2; t3]
   *    根据Eq.(37)的推导过程，可知
   *        h1 = r11*al + r12*be + r13 + t1*rh
   *        h2 = r21*al + r22*be + r23 + t2*rh
   *        h3 = r31*al + r32*be + r33 + t3*rh
   *    又有
   *        z1 = h1/h3
   *        z2 = h2/h3
   *    可知待求的Jacobian为两行，第一行为z1关于[al; be; rh]的Jacobian，第二行为z2关于[al; be; rh]的Jacobian。
   *    利用分式求导法则，有
   *        z1' = (h1'*h3-h1*h3')/(h3*h3)
   *        z2' = (h2'*h3-h2*h3')/(h3*h3)
   *    令v=[al; be; rh]，于是有：
   *        dz1/dv = [(dh1/dv)*h3-h1*(dh3/dv)]/(h3*h3) = 1/h3*(dh1/dv) - h1/(h3*h3)*(dh3/dv)
   *        dz2/dv = [(dh2/dv)*h3-h2*(dh3/dv)]/(h3*h3) = 1/h3*(dh2/dv) - h2/(h3*h3)*(dh3/dv)
   *    根据前面h1、h2和h3的解析式，可知：
   *        dh1/dv = [r11, r12, t1]
   *        dh2/dv = [r21, r22, t2]
   *        dh3/dv = [r31, r32, t3]
   *    根据W阵的组成形式：
   *        W = [R(:,1), R(:,2), t.trans]
   *          = [r11, r12, t1;
   *             r21, r22, t2;
   *             r31, r32, t3]
   *    可知：
   *        W.row(0) = dh1/dv
   *        W.row(1) = dh2/dv
   *        W.row(2) = dh3/dv
   *    于是有Jacobian表达式如下：
   *        J.row(0) = 1/h3*W.row(0) - h1/(h3*h3)*W.row(2);
   *        J.row(1) = 1/h3*W.row(1) - h2/(h3*h3)*W.row(2);
   */

  // Compute the residual.
  Eigen::Vector2d z_hat(h1/h3, h2/h3);
  r = z_hat - z;

  // Compute the weight based on the residual.
  double e = r.norm();
  if (e <= optimization_config.huber_epsilon)
    w = 1.0;
  else
    w = optimization_config.huber_epsilon / (2*e);      // QXC：这个权将始终比0.5小

  return;
}

// 利用三角化的方式求解c1下特征点的深度，可参见《14讲》P154-(7.25)式（需要把x1和x2的位置互换一下），深度d是该式的前两行构成的最小二乘解：d*(x2^)*R12*x1=-(x2^)*t
void Feature::generateInitialGuess(
    const Eigen::Isometry3d& T_c1_c2, const Eigen::Vector2d& z1,
    const Eigen::Vector2d& z2, Eigen::Vector3d& p) const {
  // Construct a least square problem to solve the depth.
  Eigen::Vector3d m = T_c1_c2.linear() * Eigen::Vector3d(z1(0), z1(1), 1.0);

  Eigen::Vector2d A(0.0, 0.0);
  A(0) = m(0) - z2(0)*m(2);
  A(1) = m(1) - z2(1)*m(2);

  Eigen::Vector2d b(0.0, 0.0);
  b(0) = z2(0)*T_c1_c2.translation()(2) - T_c1_c2.translation()(0);
  b(1) = z2(1)*T_c1_c2.translation()(2) - T_c1_c2.translation()(1);

  // Solve for the depth.
  double depth = (A.transpose() * A).inverse() * A.transpose() * b;
  p(0) = z1(0) * depth;
  p(1) = z1(1) * depth;
  p(2) = depth;
  return;
}

// 查看当前特征在首末两帧之间的视差是否够大（或者首末两帧间的位移是否足够远）
bool Feature::checkMotion(
    const CamStateServer& cam_states) const {

  const StateIDType& first_cam_id = observations.begin()->first;
  const StateIDType& last_cam_id = (--observations.end())->first;

  Eigen::Isometry3d first_cam_pose;       // QXC：观测到该特征点的首帧位姿
  const Eigen::Vector4d& first_cam_orientation = cam_states.find(first_cam_id)->second.orientation;
  first_cam_pose.linear() = Eigen::Quaterniond(
      first_cam_orientation(3),first_cam_orientation(0),first_cam_orientation(1),first_cam_orientation(2)).toRotationMatrix();
  first_cam_pose.translation() =
      cam_states.find(first_cam_id)->second.position;

  Eigen::Isometry3d last_cam_pose;       // QXC：观测到该特征点的末帧位姿
  const Eigen::Vector4d& last_cam_orientation = cam_states.find(last_cam_id)->second.orientation;
  last_cam_pose.linear() = Eigen::Quaterniond(
      last_cam_orientation(3),last_cam_orientation(0),last_cam_orientation(1),last_cam_orientation(2)).toRotationMatrix();
  last_cam_pose.translation() =
      cam_states.find(last_cam_id)->second.position;

  // Get the direction of the feature when it is first observed.
  // This direction is represented in the world frame.
  Eigen::Vector3d feature_direction(
      observations.begin()->second(0),
      observations.begin()->second(1), 1.0);
  feature_direction = feature_direction / feature_direction.norm();
  feature_direction = first_cam_pose.linear()*feature_direction;        // QXC：计算该特征点在首帧下的观测单位矢量，求出其世界坐标系下的坐标

  // Compute the translation between the first frame
  // and the last frame. We assume the first frame and
  // the last frame will provide the largest motion to
  // speed up the checking process.
  Eigen::Vector3d translation = last_cam_pose.translation() -
    first_cam_pose.translation();
  double parallel_translation =
    translation.transpose()*feature_direction;    // QXC：计算translation在feature_direction方向上的投影长度
  Eigen::Vector3d orthogonal_translation = translation -
    parallel_translation*feature_direction;       // QXC：以首帧为参照，计算末帧位置到feature_direction方向垂点到末帧的矢量，可以理解为一种衡量视差大小的方式

  if (orthogonal_translation.norm() >
      optimization_config.translation_threshold)  // QXC：这个垂线距离大于阈值有两种可能，一种是视差比较小，另一种是首末两帧足够远
    return true;
  else return false;
}

// 用Mour07中的Appendix中给出的方法计算feature的位置：先用观测到feature的首帧和末帧三角化一个初值，然后用LM法，以所有帧重投影误差为残差求最小二乘解
bool Feature::initializePosition(
    const CamStateServer& cam_states) {
  // Organize camera poses and feature observations properly.
  std::vector<Eigen::Isometry3d,
    Eigen::aligned_allocator<Eigen::Isometry3d> > cam_poses(0);     // QXC：Isometry3d是三维坐标变换阵，即包含了旋转量R和位移量t
  std::vector<Eigen::Vector2d,
    Eigen::aligned_allocator<Eigen::Vector2d> > measurements(0);

  for (auto& m : observations) {                                    // QXC：获得当前feature的所有观测，及观测到它时的各帧绝对位姿
    // TODO: This should be handled properly. Normally, the
    //    required camera states should all be available in
    //    the input cam_states buffer.
    auto cam_state_iter = cam_states.find(m.first);
    if (cam_state_iter == cam_states.end()) continue;

    // Add the measurement.
    measurements.push_back(m.second.head<2>());

    // This camera pose will take a vector from this camera frame
    // to the world frame.
    Eigen::Isometry3d cam_pose;
    const Eigen::Vector4d& cam_qua = cam_state_iter->second.orientation;
    cam_pose.linear() = Eigen::Quaterniond(
        cam_qua(3),cam_qua(0),cam_qua(1),cam_qua(2)).toRotationMatrix();
    cam_pose.translation() = cam_state_iter->second.position;

    cam_poses.push_back(cam_pose);
  }

  // All camera poses should be modified such that it takes a
  // vector from the first camera frame in the buffer to this
  // camera frame.
  Eigen::Isometry3d T_c0_w = cam_poses[0];
  for (auto& pose : cam_poses)
    pose = pose.inverse() * T_c0_w;     // QXC：将所有位姿转换到首帧相机坐标系下（获得相对位姿）

  // Generate initial guess
  Eigen::Vector3d initial_position(0.0, 0.0, 0.0);
  generateInitialGuess(cam_poses[cam_poses.size()-1], measurements[0],
      measurements[measurements.size()-1], initial_position);       // QXC：利用当前feature在观测到它的首帧和末帧中的相机系坐标，及首帧末帧间的相对位姿，进行三角化求深度
  Eigen::Vector3d solution(
      initial_position(0)/initial_position(2),
      initial_position(1)/initial_position(2),
      1.0/initial_position(2));     // QXC：逆深度参数化

  // Apply Levenberg-Marquart method to solve for the 3d position.
  double lambda = optimization_config.initial_damping;
  int inner_loop_cntr = 0;
  int outer_loop_cntr = 0;
  bool is_cost_reduced = false;
  double delta_norm = 0;

  // Compute the initial cost.      // QXC：这里的cost是feature在观测到它的每一帧的重投影误差，用首帧下的三角化结果（initial guess）以及某帧和首帧间的相对位姿进行投影
  double total_cost = 0.0;
  for (int i = 0; i < cam_poses.size(); ++i) {
    double this_cost = 0.0;
    cost(cam_poses[i], solution, measurements[i], this_cost);
    total_cost += this_cost;
  }

  // Outer loop.
  do {
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    for (int i = 0; i < cam_poses.size(); ++i) {
      Eigen::Matrix<double, 2, 3> J;
      Eigen::Vector2d r;
      double w;

      jacobian(cam_poses[i], solution, measurements[i], J, r, w);   // QXC：计算当前feature在第i帧的重投影误差关于逆深度参数的Jacobian

      if (w == 1) {
        A += J.transpose() * J;     // QXC：每个重投影误差为最小二乘累加指标中的一项残差，根据Jacobian的公式，总残差序列的J.trans*J将等于各残差项的j.trans*j之和
        b += J.transpose() * r;
      } else {
        double w_square = w * w;
        A += w_square * J.transpose() * J;
        b += w_square * J.transpose() * r;
      }
    }

    // Inner loop.
    // Solve for the delta that can reduce the total cost.
    do {
      Eigen::Matrix3d damper = lambda * Eigen::Matrix3d::Identity();
      Eigen::Vector3d delta = (A+damper).ldlt().solve(b);       // QXC：求解增量
      Eigen::Vector3d new_solution = solution - delta;
      delta_norm = delta.norm();

      double new_cost = 0.0;
      for (int i = 0; i < cam_poses.size(); ++i) {
        double this_cost = 0.0;
        cost(cam_poses[i], new_solution, measurements[i], this_cost);
        new_cost += this_cost;
      }

      if (new_cost < total_cost) {
        is_cost_reduced = true;
        solution = new_solution;
        total_cost = new_cost;
        lambda = lambda/10 > 1e-10 ? lambda/10 : 1e-10;     // QXC：当更新后的状态获得了更小的cost时，缩小lambda，使得更接近于高斯牛顿法
      } else {
        is_cost_reduced = false;
        lambda = lambda*10 < 1e12 ? lambda*10 : 1e12;       // QXC：当更新后的状态没获得更小的cost时，增大lambda，使得更接近于最速下降法
      }

    } while (inner_loop_cntr++ <
        optimization_config.inner_loop_max_iteration && !is_cost_reduced);

    inner_loop_cntr = 0;

  } while (outer_loop_cntr++ <
      optimization_config.outer_loop_max_iteration &&
      delta_norm > optimization_config.estimation_precision);

  // Covert the feature position from inverse depth
  // representation to its 3d coordinate.
  Eigen::Vector3d final_position(solution(0)/solution(2),
      solution(1)/solution(2), 1.0/solution(2));

  // Check if the solution is valid. Make sure the feature
  // is in front of every camera frame observing it.
  bool is_valid_solution = true;
  for (const auto& pose : cam_poses) {
    Eigen::Vector3d position =
      pose.linear()*final_position + pose.translation();
    if (position(2) <= 0) {
      is_valid_solution = false;
      break;
    }
  }

  // Convert the feature position to the world frame.
  position = T_c0_w.linear()*final_position + T_c0_w.translation();

  if (is_valid_solution)
    is_initialized = true;

  return is_valid_solution;
}
} // namespace msckf_vio

#endif // MSCKF_VIO_FEATURE_H
