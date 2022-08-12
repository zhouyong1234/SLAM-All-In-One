#ifndef GLOBAL_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define GLOBAL_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "global_localization/sensor_data/cloud_data.hpp"

namespace global_localization {
class RegistrationInterface {
  public:
    virtual ~RegistrationInterface() = default;

    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                          const Eigen::Matrix4f& predict_pose, 
                          CloudData::CLOUD_PTR& result_cloud_ptr,
                          Eigen::Matrix4f& result_pose) = 0;
    virtual float GetFitnessScore() = 0;
};
} 

#endif