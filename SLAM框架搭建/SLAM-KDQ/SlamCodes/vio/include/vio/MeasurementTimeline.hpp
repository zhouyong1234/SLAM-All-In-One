//
// Created by kdq on 2021/5/26.
//
#pragma once
#include "eigen3/Eigen/Eigen"

struct IMU {
  IMU() {
    timestamp_ = -0.001;
    acc_.setZero();
    gyro_.setZero();
  }
  void operator = (const IMU& imu) {
    timestamp_ = imu.timestamp_;
    acc_ = imu.acc_;
    gyro_ = imu.gyro_;
  }
  void print() {
    std::cout << "[IMU]:timestamp = " << timestamp_ << " acc = " << acc_ << " gyro = " << gyro_ << std::endl;
  }

  IMU(double t,const Eigen::Vector3d& acc,const Eigen::Vector3d& gyro) {
    timestamp_ = t;
    acc_ = acc;
    gyro_ = gyro;
  }
  double timestamp_;
  Eigen::Vector3d acc_;
  Eigen::Vector3d gyro_;
};

template<typename Meas>
class MeasurementTimeline {
 public:
  MeasurementTimeline() {
  }
  void clear() {
    measMap_.clear();
  };
  void addMeas(double t,const Meas& meas) {
    measMap_[t] = meas;
  }
  void clean(double t) {
    while (!measMap_.empty() && measMap_.begin()->first <= t) {
      measMap_.erase(measMap_.begin());
    }
  }
  bool empty() const {
    return measMap_.empty();
  }

  typename std::map<double,Meas>::const_iterator getLowIter(double t) const {
    typename std::map<double,Meas>::const_iterator it = measMap_.lower_bound(t);
    it--;
    if (!measMap_.count(it->first)) {
      it = measMap_.end();
    }
    return it;
  }
  typename std::map<double,Meas>::const_iterator getHighIter(double t) const {
    typename std::map<double,Meas>::const_iterator it = measMap_.upper_bound(t);
    return it;
  }

  std::map<double,Meas> measMap_;
 private:

};
