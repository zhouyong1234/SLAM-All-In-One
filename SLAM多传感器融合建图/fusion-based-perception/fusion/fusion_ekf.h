// Copyright 2021 Sui Fang

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>

#include "state.h"
#include "kalman_filter.h"

namespace kit {
namespace perception {
namespace fusion {

class FusionEKF {
  public:
    /**
     * Constructor.
     */
    FusionEKF();

    /**
     * Destructor.
     */
    virtual ~FusionEKF();

    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    void UpdateMotion(const State &state, const Measurement &mea);
    void UpdateAttr(const State &state, const Measurement &mea);
    Eigen::VectorXd GetState();
    Eigen::VectorXd GetAttrState();

    // Kalman Filter update and prediction for motion 
    KalmanFilter motion_filter_;

    // Kalman Filter update and prediction for atrributes
    KalmanFilter attr_filter_;

  private:
    /**
     * A helper method to calculate RMSE.
     */
    Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

    /**
     * A helper method to calculate Jacobians.
     */
    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

    // check whether the tracking toolbox was initiallized or not (first measurement)
    bool is_motion_filter_initialized_;
    bool is_attr_filter_initialized_;

    // previous timestamp
    long long previous_motion_timestamp_;
    long long previous_attr_timestamp_;

    Eigen::MatrixXd R_laser_;    // laser measurement noise
    Eigen::MatrixXd R_radar_;    // radar measurement noise
    Eigen::MatrixXd R_camera_;   // camera measurement noise
    Eigen::MatrixXd H_laser_;    // measurement function for laser
    Eigen::MatrixXd H_camera_;    // measurement function for camera
    Eigen::MatrixXd H_radar_jacobian_;         // measurement function for radar
    Eigen::MatrixXd F_motion_;      // state transition matrix for lidar and radar
    Eigen::MatrixXd F_camera_;      // state transition matrix for camera
    Eigen::MatrixXd P_;             // state covariance matrix for all

    float noise_ax;
    float noise_ay;
    float noise_attr_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace kit
