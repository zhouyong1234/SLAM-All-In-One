#include "fusion_ekf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

namespace kit {
namespace perception {
namespace fusion {

FusionEKF::FusionEKF() {
    is_motion_filter_initialized_ = false;
    is_attr_filter_initialized_ = false;

    previous_motion_timestamp_ = 0;
    previous_attr_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(5, 5);
    R_camera_ = MatrixXd(5, 5);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(5, 5);
    H_camera_ = MatrixXd(5, 5);
    H_radar_jacobian_ = MatrixXd(3, 4);
    F_motion_ = MatrixXd(5, 5);
    F_camera_ = MatrixXd(5, 5);
    P_ = MatrixXd(5, 5);

    // measurement covariance matrix - laser
    R_laser_ << 0.0225, 0, 0, 0, 0,
             0, 0.0225, 0, 0, 0,
             0, 0, 0.0225, 0, 0,
             0, 0, 0, 0.0225, 0,
             0, 0, 0, 0, 5;
    // measurement covariance matrix - camera
    R_camera_ << 0.0225, 0, 0, 0, 0,
             0, 0.0225, 0, 0, 0,
             0, 0, 0.0225, 0, 0,
             0, 0, 0, 0.0225, 0,
             0, 0, 0, 0, 0.0225;

    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
             0, 0.0009, 0,
             0, 0, 0.09;

    // TODO: Finish initializing the FusionEKF. Set the process and measurement noises

    // measurement matrix - laser 
    H_laser_ << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0,
             0, 0, 1, 0, 0,
             0, 0, 0, 1, 0,
             0, 0, 0, 0, 1;

    // measurement matrix - camera 
    H_camera_ << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0,
             0, 0, 1, 0, 0,
             0, 0, 0, 1, 0,
             0, 0, 0, 0, 1;

    // state covariance matrix for all
    P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

    // state transition matrix for laser and radar

    F_motion_ << 1, 0, 1, 0, 0,
        0, 1, 0, 1, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

    F_camera_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

    // set measurement noises
    noise_ax = 9;
    noise_ay = 9;
    noise_attr_ = 5;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::UpdateAttr(const State &state, const Measurement &mea) {
    attr_filter_.x_ = VectorXd(5);
    attr_filter_.F_ = F_camera_;
    attr_filter_.P_ = P_;
    if (mea.sensor_type == SensorType::CAMERA) {
        attr_filter_.x_ << state.meas[0], state.meas[1], state.meas[2], state.meas[3], state.meas[4];
    } else {
        return;
    }

    float dt = std::abs(mea.time_ns - state.time_ns);  //  in seconds
    attr_filter_.Q_ = MatrixXd(5, 5);
    attr_filter_.Q_ << dt*noise_attr_, 0, 0, 0, 0,
        0, dt*noise_attr_, 0, 0, 0,
        0, 0, dt*noise_attr_, 0, 0,
        0, 0, 0, dt*noise_attr_, 0,
        0, 0, 0, 0, dt*noise_attr_;
    attr_filter_.Predict();
    attr_filter_.H_ = H_camera_;
    attr_filter_.R_ = R_camera_;
    attr_filter_.Update(mea.meas);
}

void FusionEKF::UpdateMotion(const State &state, const Measurement &measurement) {
    /*****************************************************************************
     * Initialization 
     ****************************************************************************/
    motion_filter_.x_ = VectorXd(5);
    motion_filter_.P_ = P_;

    if (measurement.sensor_type == SensorType::RADAR) {
        // Convert radar from polar to cartesian coordinates and initialize state.
        // float rho = state.meas[0];      // range: radial distance from origin
        // float phi = state.meas[1];      // bearing: angle between rho and x axis
        // float rho_dot = state.meas[2];  // radial velocity: change of rho
        // motion_filter_.x_ << rho * cos(phi), rho * sin(phi), rho_dot * cos(phi), rho_dot * sin(phi);
        // phi is not the direction of the speed, it is better to set vx and vy to 0
        // motion_filter_.x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;  // x, y, vx, vy, class

        // cartesian coordinate frame
        // TODO: Initialize state.

    } else if (measurement.sensor_type == SensorType::LIDAR) {
        // TODO: Initialize state.
        
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
     * Update the state transition matrix F according to the new elapsed time.
     - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */

    // compute the time elapsed between the current and previous measurements
    // float dt = (measurement.time_ns - previous_motion_timestamp_) / 1000000.0;  //  in seconds
    float dt = measurement.time_ns - state.time_ns;  //  in seconds
    // previous_motion_timestamp_ = measurement.time_ns;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    // TODO: Modify the F matrix so that the time is integrated


    // TODO: set the process covariance matrix Q


    motion_filter_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    // TODO:
    /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     */

}

VectorXd FusionEKF::GetState() {
    return motion_filter_.x_;
}

VectorXd FusionEKF::GetAttrState() {
    return attr_filter_.x_;
}

VectorXd FusionEKF::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
     * Calculate the RMSE here.
     */

    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size() || estimations.size() == 0){
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }

    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd FusionEKF::CalculateJacobian(const VectorXd& x_state) {
    /**
     * Calculate a Jacobian here.
     */
    MatrixXd Hj(3,4);
    Hj << 0,0,0,0,
       0,0,0,0,
       0,0,0,0;

    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    //check division by zero
    if(fabs(c1) < 0.0001){
        std::cout << "Function CalculateJacobian() has Error: Division by Zero" << std::endl;
        return Hj;
    }

    //compute the Jacobian matrix
    Hj << (px/c2),                (py/c2),                0,      0,
       -(py/c1),               (px/c1),                0,      0, 
       py*(vx*py - vy*px)/c3,  px*(px*vy - py*vx)/c3,  px/c2,  py/c2;

    return Hj;
}

}  // namespace fusion
}  // namespace perception
}  // namespace kit
