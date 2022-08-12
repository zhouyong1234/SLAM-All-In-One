#include <FilterFusion/Propagator.h>

#include <glog/logging.h>

namespace FilterFusion {

Propagator::Propagator(const double kl, const double kr,const double b, const double noise_factor) {
    wheel_propagator_ = std::make_unique<TGK::WheelProcessor::WheelPropagator>(kl, kr, b, noise_factor);
}

void Propagator::Propagate(const double begin_wl, const double begin_wr,
                           const double end_wl, const double end_wr,
                           State* state) {
    // Propagate mean and covariance of the wheel pose state.
    Eigen::Matrix<double, 6, 6> Phi;
    Eigen::Matrix<double, 6, 6> wheel_pose_cov = state->covariance.topLeftCorner<6, 6>();
    wheel_propagator_->PropagateUsingEncoder(begin_wl, begin_wr, end_wl, end_wr, 
                                             &state->wheel_pose.G_R_O, &state->wheel_pose.G_p_O,
                                             &Phi, 
                                             &wheel_pose_cov);
    state->covariance.topLeftCorner<6, 6>() = wheel_pose_cov;
    
    // Propagate covariance of other states.
    const int cov_size = state->covariance.rows();
    const int other_size = cov_size - 6;
    if (other_size <= 0) { return; }

    state->covariance.block(0, 6, 6, other_size) = 
        Phi * state->covariance.block(0, 6, 6, other_size).eval();

    //Force symmetric.
    state->covariance = state->covariance.eval().selfadjointView<Eigen::Upper>();
}

}  // namespace FilterFusion