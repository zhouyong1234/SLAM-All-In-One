#pragma once
#include <map>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

struct IMU {
 public:
  IMU() = default;
  IMU(double t,Eigen::Vector3d acc,Eigen::Vector3d gyro,Eigen::Quaterniond qwb,double tof,int state):
  timestamp(t),
  Acc(acc),
  Gyro(gyro),
  Qwb(qwb),
  tofDist(tof),
  flyState(state){
  }
  double timestamp;
  Eigen::Vector3d Acc;
  Eigen::Vector3d Gyro;
  Eigen::Quaterniond Qwb;
  double tofDist;
  int flyState;
};

struct Image {
 public:
  Image() = default;
  Image(double t,cv::Mat& img):
  timestamp(t) {
    data = img.clone();
  }
  double timestamp;
  cv::Mat data;
};

template<typename Meas>
class MeasurementTimeline {
 public:
  typedef Meas mtMeas;
  std::map<double, mtMeas> measMap_;
  typename std::map<double, mtMeas>::iterator itMeas_;
  double maxWaitTime_;
  double minWaitTime_;

  MeasurementTimeline() {
    maxWaitTime_ = 0.1;
    minWaitTime_ = 0.0;
  };

  virtual ~MeasurementTimeline() {};

  void addMeas(const mtMeas &meas, const double &t) {
    measMap_[t] = meas;
  }

  void clear() {
    measMap_.clear();
  }

  void clean(double t) {
    while ((!measMap_.empty()) && measMap_.begin()->first <= t) {
      measMap_.erase(measMap_.begin());
    }
  }

  bool empty() {
    return measMap_.empty();
  }
  /**
   * @brief Get the next time in measurement timeline.
   * @param actualTime current time
   * @param nextTime the time after the current that you will get
   * @return true if getNextTime is successful
   */
  bool getNextTime(double actualTime, double &nextTime) {
    itMeas_ = measMap_.upper_bound(actualTime);
    if (itMeas_ != measMap_.end()) {
      nextTime = itMeas_->first;
      return true;
    } else {
      return false;
    }
  }

  void waitTime(double actualTime, double &time) {
    double measurementTime = actualTime - maxWaitTime_;
    if (!measMap_.empty() && measMap_.rbegin()->first + minWaitTime_ > measurementTime) {
      measurementTime = measMap_.rbegin()->first + minWaitTime_;
    }
    if (time > measurementTime) {
      time = measurementTime;
    }
  }

  bool getLastTime(double &lastTime) {
    if (!measMap_.empty()) {
      lastTime = measMap_.rbegin()->first;
      return true;
    } else {
      return false;
    }
  }

  bool hasMeasurementAt(double t) {
    return measMap_.count(t) > 0;
  }
};