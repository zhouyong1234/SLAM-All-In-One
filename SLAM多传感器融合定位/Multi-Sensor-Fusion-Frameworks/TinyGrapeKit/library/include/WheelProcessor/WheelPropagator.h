#pragma once

#include <Eigen/Core>

namespace TGK {
namespace WheelProcessor {

class WheelPropagator {
public:
    WheelPropagator(const double kl, const double kr, const double b, 
                    const double noise_factor = 0.02, 
                    const double roll_pitch_noise = 1e-6, 
                    const double z_noise = 0.0001);

    void PropagateUsingEncoder(const double begin_wl, const double begin_wr,
                               const double end_wl, const double end_wr,
                               Eigen::Matrix3d* G_R_O, Eigen::Vector3d* G_p_O,
                               Eigen::Matrix<double, 6, 6>* J_wrt_pose = nullptr,
                               Eigen::Matrix<double, 6, 6>* cov = nullptr);

private:
    // Intrinsic.
    double kl_;
    double kr_;
    double b_;

    // Noise.
    double noise_factor_;
    double roll_pitch_noise_;
    double z_noise_;
};

}  // namespace WheelProcessor
} // namespace TGK