#ifndef GLOBAL_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define GLOBAL_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "global_localization/scan_adjust/distortion_adjust.hpp"
#include "global_localization/sensor_data/velocity_data.hpp"
#include "global_localization/sensor_data/cloud_data.hpp"

namespace global_localization {
class DistortionAdjust {
  public:
    void SetMotionInfo(float scan_period, VelocityData velocity_data);
    bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

  private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

  private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
};
} // namespace lidar_slam
#endif