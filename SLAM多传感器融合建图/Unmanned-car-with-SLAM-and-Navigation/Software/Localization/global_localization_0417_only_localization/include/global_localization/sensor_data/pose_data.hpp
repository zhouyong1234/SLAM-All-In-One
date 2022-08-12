#ifndef GLOBAL_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_
#define GLOBAL_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace global_localization {
class PoseData {
  public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double time = 0.0;

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif