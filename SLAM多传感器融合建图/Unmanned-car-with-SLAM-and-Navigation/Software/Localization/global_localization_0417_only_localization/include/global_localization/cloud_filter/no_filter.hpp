#ifndef GLOBAL_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define GLOBAL_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_

#include "global_localization/cloud_filter/cloud_filter_interface.hpp"

namespace global_localization {
class NoFilter: public CloudFilterInterface {
  public:
    NoFilter();

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
};
}
#endif