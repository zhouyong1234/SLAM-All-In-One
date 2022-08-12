#pragma once
#include <FilterFusion/State.h>

namespace FilterFusion {

void AugmentState(const double timestamp, const long int frame_id, State* state);

}  // namespace FilterFusion