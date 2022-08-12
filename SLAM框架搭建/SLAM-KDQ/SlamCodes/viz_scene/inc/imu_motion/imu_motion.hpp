#pragma once
#include <iostream>
#include <fstream>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "utilities/kinematic.hpp"
#include "imu_motion/motion_para.hpp"

struct ImuMotionState {
  ImuMotionState() {
    timestamp_ = 0;
    pos_.setZero();
    vel_.setZero();
    acc_.setZero();
    gyr_.setZero();
    acc_bias_.setZero();
    gyr_bias_.setZero();
  }
  void printData() {
    std::cout << timestamp_ << ","
              << pos_.transpose() << ","
              << qwi_.w() << " " << qwi_.x() << " " << qwi_.y() << " " << qwi_.z() << ","
              << vel_.transpose() << ","
              << acc_.transpose() << ","
              << gyr_.transpose() << ","
              << acc_bias_.transpose() << ","
              << gyr_bias_.transpose() << std::endl;
  }
  double timestamp_;
  Eigen::Vector3d pos_;
  Eigen::Quaterniond qwi_;
  Eigen::Vector3d vel_; // velocity of imu in the world frame
  Eigen::Vector3d acc_;
  Eigen::Vector3d gyr_;
  Eigen::Vector3d acc_bias_;
  Eigen::Vector3d gyr_bias_;
};


class ImuMotion {
  private:
    ImuMotionState init_state_;
    const MotionParam* para_;
    Eigen::Vector3d gyr_bias_,acc_bias_;
  public:

    /** @brief Constructor
     *  @param motionParam - parameter class which include all variables of simulated model
    */
    ImuMotion(const MotionParam* motionParam):para_(motionParam){
      gyr_bias_.setZero();
      acc_bias_.setZero();
    };

    /** @brief Destructor
     */ 
    ~ImuMotion(){};

    /** @brief simulate imu state through sin function
     *  @param t - timestamp of model
     *  @return imu state include pos,vel,attitude and acc,gyro 
     */ 
    ImuMotionState simImuMotion(double t);
    
    /** @brief add noise setted by parameters into imu state  
     *  @param data - imu state without noise
     *  @return imu state whose gyro,acc with nose
     */ 
    ImuMotionState addImuNoise(const ImuMotionState& data);

    /** @brief propagate imu state using acc,gyro to test imu infos good or not 
     *  @param rawDataFile including acc,gyro
     *  @param outDataFile recording imu state include pose vel attitude propagated by acc gyro
     */ 
    void testImuMotionData(const std::string rawDataFile,const std::string outDataFile);
};