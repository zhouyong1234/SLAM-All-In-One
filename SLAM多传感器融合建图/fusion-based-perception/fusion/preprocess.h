#pragma once

#include <Eigen/Core>
#include "object.h"

namespace kit {
namespace perception {
namespace fusion {

void PreProcess(const LiDARObjectListPtr &lidar_obj_list, Eigen::Affine3d &extrinsic);

void PreProcess(const RadarObjectListPtr &lidar_obj_list, Eigen::Affine3d &extrinsic);

}  // namespace fusion
}  // namespace perception
}  // namespace kit
