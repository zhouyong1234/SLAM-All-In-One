#pragma once

#include <memory>

#include <TGK/WheelProcessor/WheelPropagator.h>
#include <FilterFusion/State.h>

namespace FilterFusion {

class Propagator {
public:
    Propagator(const double kl, const double kr,const double b, const double noise_factor);

    void Propagate(const double begin_wl, const double begin_wr,
                   const double end_wl, const double end_wr,
                   State* state);
private:
    std::unique_ptr<TGK::WheelProcessor::WheelPropagator> wheel_propagator_;
};

}  // namespace FilterFusion