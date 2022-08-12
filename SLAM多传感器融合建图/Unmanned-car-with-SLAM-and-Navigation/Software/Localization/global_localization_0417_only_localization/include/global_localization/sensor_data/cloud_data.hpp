#ifndef GLOBAL_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define GLOBAL_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace global_localization {
class CloudData {
  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData()
      :cloud_ptr(new CLOUD()) {
    }

  public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
}

#endif