#pragma once

#include <FilterFusion/State.h>

namespace FilterFusion {

class Initializer {
public:
    Initializer(const Eigen::Matrix3d& O_R_C, const Eigen::Vector3d& O_p_C, 
                const double kl, const double kr, const double b);

    void Initialize(const double timestamp, State* init_state);

private:
    Eigen::Matrix3d O_R_C_;
    Eigen::Vector3d O_p_C_;

    double kl_;
    double kr_;
    double b_;
};

}  // namespace FilterFusion