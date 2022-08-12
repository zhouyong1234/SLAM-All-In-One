#include "mutex"
#include "MeasurementTimeLine.hpp"
#include "ImageAlignment.hpp"


class OpticalFlowEstimation {
 public:
  OpticalFlowEstimation() {

  }

  void updateEstimate();

  void addImuMeasurement(const IMU& data) {
    std::lock_guard<std::mutex> lck(imuMutex);
    imuMeas_.measMap_[data.timestamp] = data;
  }

  void addImageMeasurement(const Image& data) {
    std::lock_guard<std::mutex> lck(imageMutex);
    imageMeas_.measMap_[data.timestamp] = data;
  }
 private:
  std::mutex imuMutex;
  std::mutex imageMutex;
  ImageAlignment* imgAligner_;
  MeasurementTimeline<IMU> imuMeas_;
  MeasurementTimeline<Image> imageMeas_;

};