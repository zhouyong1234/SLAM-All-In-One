#include <FilterFusion/StateMarginalizer.h>

namespace FilterFusion {

void MargOldestState(State* state) {
    if (state->camera_frames.empty()) { return; }
    
    // Remove camera state.
    int org_state_idx = state->camera_frames.front()->state_idx;
    int state_idx = org_state_idx;
    state->camera_frames.pop_front();
    for (auto& cam_fm : state->camera_frames) {
        cam_fm->state_idx = state_idx;
        state_idx += cam_fm->size;
    }

    // Remove row and col in covariance matrix.
    const int old_cov_size = state->covariance.rows();
    const int new_cov_size = old_cov_size - 6;
    Eigen::MatrixXd new_cov(new_cov_size, new_cov_size);

    const Eigen::MatrixXd& old_cov = state->covariance;
    new_cov.block<6, 6>(0, 0) = old_cov.block<6, 6>(0, 0);
    new_cov.block(0, 6, 6, new_cov_size - 6) = old_cov.block(0, 12, 6, new_cov_size - 6);
    new_cov.block(6, 6, new_cov_size - 6, new_cov_size - 6)
        = old_cov.block(12, 12, new_cov_size - 6, new_cov_size - 6);

    // Force symetric.
    state->covariance = new_cov.selfadjointView<Eigen::Upper>();
}

}  // namespace FilterFusion