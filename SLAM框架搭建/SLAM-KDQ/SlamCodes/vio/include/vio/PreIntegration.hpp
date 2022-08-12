//
// Created by kdq on 2021/5/26.
//
//
#pragma once
#include "kindr/Core"
#include "MeasurementTimeline.hpp"
class PreIntegration {
 public:
  PreIntegration(Eigen::Vector3d ba,Eigen::Vector3d bg) {
    dP_.setZero();
    dQ_.setIdentity();
    dV_.setZero();
    ba_ = ba;
    bg_ = bg;
  };

  void fix() {
    dQ_.fix();
  }
  void midIntegration(const IMU& imu) {
    if (imuLast_.timestamp_ < 0) {
      imuLast_ = imu;
      return;
    }
    double dt = imu.timestamp_ - imuLast_.timestamp_;
    Eigen::Vector3d accLast = imuLast_.acc_ - ba_;
    Eigen::Vector3d accNow = imu.acc_ - ba_;
    Eigen::Vector3d gyrLast = imuLast_.gyro_ - bg_;
    Eigen::Vector3d gyrNow = imu.gyro_ - bg_;
    kindr::RotationQuaternionPD lastQ = dQ_;
    Eigen::Vector3d dAngle = (gyrLast + gyrNow) * dt * 0.5;
    kindr::RotationMatrixD dq = dq.exponentialMap(dAngle);
    dQ_ = dQ_ * dq;
    Eigen::Vector3d acc = (lastQ.rotate(accLast) + dQ_.rotate(accNow)) * 0.5;
    dP_ += dV_ * dt + 0.5 * acc * dt * dt;
    dV_ += acc * dt;
    imuLast_ = imu;
  }

  Eigen::Matrix3d rotationMatrix() const{
    return dQ_.toImplementation().toRotationMatrix();
  }

  Eigen::Vector3d eulerAngle() const {
    kindr::EulerAnglesZyxD euler(dQ_);
    euler.setUnique(); //设置唯一的转换方式，不然有可能变成很大的值
    return euler.toImplementation();
  }

 private:
  IMU imuLast_;
  Eigen::Vector3d dP_;
  kindr::RotationQuaternionPD dQ_;
  Eigen::Vector3d dV_;
  Eigen::Vector3d ba_;
  Eigen::Vector3d bg_;
};
