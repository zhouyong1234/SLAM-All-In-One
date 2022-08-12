#ifndef GLOBAL_LOCALIZATION_MODELS_CLOUD_FILTER_BOX_FILTER_HPP_
#define GLOBAL_LOCALIZATION_MODELS_CLOUD_FILTER_BOX_FILTER_HPP_

#include <pcl/filters/crop_box.h>
#include "global_localization/cloud_filter/cloud_filter_interface.hpp"

namespace global_localization {
class BoxFilter: public CloudFilterInterface {
  public:
    BoxFilter(YAML::Node node);
    BoxFilter() = default;

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

    void SetSize(std::vector<float> size);
    void SetOrigin(std::vector<float> origin);
    std::vector<float> GetEdge();

  private:
    void CalculateEdge();

  private:
    pcl::CropBox<CloudData::POINT> pcl_box_filter_;

    std::vector<float> origin_;
    std::vector<float> size_;
    std::vector<float> edge_;
};
}

#endif 