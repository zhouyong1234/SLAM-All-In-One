#pragma once

#include <deque>

#include <glog/logging.h>

#include <TGK/Const/Const.h>
#include <FilterFusion/StateBlock.h>

namespace FilterFusion {

struct State {
    double timestamp;

    // Wheel Instrinsic.
    WheelIntrinsic wheel_intrinsic;
    // Extrinsic.
    Extrinsic extrinsic;
    // The current wheel pose.
    WheelPose wheel_pose;
    // Camera poses.
    std::deque<CameraFramePtr> camera_frames;

    // Covariance.
    Eigen::MatrixXd covariance;

    void Update(const Eigen::VectorXd& delta_x) {
        // Update Wheel_pose.
        wheel_pose.Update(delta_x.segment(wheel_pose.state_idx, wheel_pose.size));

        // Update Camera pose.
        for (CameraFramePtr& cam_fm : camera_frames) {
            cam_fm->Update(delta_x.segment(cam_fm->state_idx, cam_fm->size));
        }
    }
};

}  // namespace FilterFusion