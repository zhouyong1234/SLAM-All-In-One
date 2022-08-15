#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeographicLib/LocalCartesian.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>

cv::RNG rng;
double w_sigma = 2.0;

namespace cg {

constexpr int kMeasDim = 3;

struct GpsData {
  double timestamp;

  Eigen::Vector3d lla;  // Latitude in degree, longitude in degree, and altitude in meter
  Eigen::Matrix3d cov;  // Covariance in m^2
};
using GpsDataPtr = std::shared_ptr<GpsData>;
using GpsDataConstPtr = std::shared_ptr<const GpsData>;

class GNSS : public Observer {
 public:
  GNSS()
  {
    file_gps_.open("/home/touchair/test/fusion_gps.txt");
  }

  virtual ~GNSS() {}

  void set_params(GpsDataConstPtr gps_data_ptr, const Eigen::Vector3d &I_p_Gps = Eigen::Vector3d::Zero()) {
    init_lla_ = gps_data_ptr->lla;
    I_p_Gps_ = I_p_Gps;
  }

  virtual Eigen::MatrixXd measurement_function(const Eigen::MatrixXd &mat_x) {
    Eigen::Isometry3d Twb;
    Twb.matrix() = mat_x;

    // std::cout << Twb.matrix() << std::endl;

    // std::cout << Twb * I_p_Gps_ << std::endl;

    return Twb * I_p_Gps_;
  }

  virtual Eigen::MatrixXd measurement_residual(const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z) {

    // std::cout << I_p_Gps_ << std::endl;

    return mat_z - measurement_function(mat_x);
  }

  virtual Eigen::MatrixXd measurement_jacobian(const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z) {
    Eigen::Isometry3d Twb;
    Twb.matrix() = mat_x;

    Eigen::Matrix<double, kMeasDim, kStateDim> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(0, 6) = -Twb.linear() * Utils::skew_matrix(I_p_Gps_);

    // std::cout << H << std::endl;

    return H;
  }

  virtual void check_jacobian(const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z) {}

  /**
   * @brief global to local coordinate, convert WGS84 to ENU frame
   *
   * @param gps_data_ptr
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d g2l(GpsDataConstPtr gps_data_ptr) {
    Eigen::Vector3d p_G_Gps;
    GNSS::lla2enu(init_lla_, gps_data_ptr->lla, &p_G_Gps);

    // p_G_Gps[0] += rng.gaussian(w_sigma);
    // p_G_Gps[1] += rng.gaussian(w_sigma);
    // p_G_Gps[2] += rng.gaussian(w_sigma);

    file_gps_ << std::fixed << std::setprecision(15) << gps_data_ptr->timestamp << " " << p_G_Gps[0] << " " << p_G_Gps[1] << " " << p_G_Gps[2] << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;

    // file_gps_ << p_G_Gps[0] << ", " << p_G_Gps[1] << ", " << p_G_Gps[2] << std::endl;

    return p_G_Gps;
  }

  /**
   * @brief local to glocal coordinate, convert ENU to WGS84 lla
   *
   * @param p_wb
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d l2g(const Eigen::Vector3d &p_wb) {
    Eigen::Vector3d lla;
    GNSS::enu2lla(init_lla_, p_wb, &lla);
    return lla;
  }

  static inline void lla2enu(const Eigen::Vector3d &init_lla,
                             const Eigen::Vector3d &point_lla,
                             Eigen::Vector3d *point_enu) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2), point_enu->data()[0], point_enu->data()[1],
                            point_enu->data()[2]);
  }

  static inline void enu2lla(const Eigen::Vector3d &init_lla,
                             const Eigen::Vector3d &point_enu,
                             Eigen::Vector3d *point_lla) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2), point_lla->data()[0], point_lla->data()[1],
                            point_lla->data()[2]);
  }

 private:
  Eigen::Vector3d init_lla_;
  Eigen::Vector3d I_p_Gps_ = Eigen::Vector3d::Zero();

  std::ofstream file_gps_;
};
using GNSSPtr = std::shared_ptr<GNSS>;

}  // namespace cg