#pragma once

#include <Eigen/Core>
#include "object.h"

namespace kit {
namespace perception {
namespace fusion {

class Predictor {
 public:
    Predictor() = default;
    bool Predict(const FusionObjectListPtr &fusion_obj_list, double ts);

}; // class Predictor

}  // namespace fusion
}  // namespace perception
}  // namespace kit
