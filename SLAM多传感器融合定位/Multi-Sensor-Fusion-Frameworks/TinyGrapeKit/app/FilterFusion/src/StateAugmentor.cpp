#include <FilterFusion/StateAugmentor.h>
#include <TGK/Util/Util.h>

namespace FilterFusion {

/// Causion: This function does not process frame id
void AugmentState(const double timestamp, const long int frame_id, State* state) {
    const CameraFramePtr cam_frame = std::make_shared<CameraFrame>();
    cam_frame->timestamp = timestamp;
    cam_frame->id = frame_id;
    
    // Compute mean.
    // 通过odometry位姿计算相机位姿
    cam_frame->G_R_C = state->wheel_pose.G_R_O * state->extrinsic.O_R_C;
    cam_frame->G_p_C = state->wheel_pose.G_p_O + state->wheel_pose.G_R_O * state->extrinsic.O_p_C;

    // Set index.
    cam_frame->state_idx = state->covariance.rows();

    // Push to state vector.
    state->camera_frames.push_back(cam_frame);

    // Extend covaraicne.
    const int old_size = state->covariance.rows();
    const int new_size = old_size + 6;
    state->covariance.conservativeResize(new_size, new_size);
    state->covariance.block(old_size, 0, 6, new_size).setZero();
    state->covariance.block(0, old_size, new_size, 6).setZero();

    /// Compute covariance.
    // 相机位姿相对于增广之前状态的雅克比
    Eigen::Matrix<double, 6, 6> J_wrt_wheel_pose;
    J_wrt_wheel_pose << state->extrinsic.O_R_C.transpose(), Eigen::Matrix3d::Zero(),
                        -state->wheel_pose.G_R_O * TGK::Util::Skew(state->extrinsic.O_p_C), 
                        Eigen::Matrix3d::Identity();

    const Eigen::Matrix<double, 6, 6> cov11 = state->covariance.block<6, 6>(0, 0);
    state->covariance.block<6, 6>(old_size, old_size) = J_wrt_wheel_pose * cov11 * J_wrt_wheel_pose.transpose();
     
    const auto& cov_top_rows = state->covariance.block(0, 0, 6, old_size);
    // New lower line.
    state->covariance.block(old_size, 0, 6, old_size) = J_wrt_wheel_pose * cov_top_rows;

    // Force symmetric.
    state->covariance = state->covariance.eval().selfadjointView<Eigen::Lower>();
}

}  // namespace FilterFusion