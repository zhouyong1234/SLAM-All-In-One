#ifndef GLOBAL_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define GLOBAL_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "global_localization/sensor_data/cloud_data.hpp"

namespace global_localization {
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
};
}

#endif