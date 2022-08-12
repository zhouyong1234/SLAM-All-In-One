#ifndef GLOBAL_LOCALIZATION_TOOLS_PRINT_INFO_HPP_
#define GLOBAL_LOCALIZATION_TOOLS_PRINT_INFO_HPP_

#include <cmath>
#include <string>
#include <Eigen/Dense>
#include "pcl/common/eigen.h"

namespace global_localization {
class PrintInfo {
  public:
    static void PrintPose(std::string head, Eigen::Matrix4f pose);
};
}
#endif