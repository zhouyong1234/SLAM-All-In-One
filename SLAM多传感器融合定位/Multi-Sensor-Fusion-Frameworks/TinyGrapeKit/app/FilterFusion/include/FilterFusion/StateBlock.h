#pragma once

#include <memory>
#include <unordered_map>
#include <Eigen/Geometry>

namespace FilterFusion {

inline void UpdateSE3(const Eigen::Matrix<double, 6, 1>& delta_x, Eigen::Matrix3d* R, Eigen::Vector3d* p) {
    // Update Rotation.
    const double delta_angle = delta_x.segment<3>(0).norm();
    Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
    if (std::abs(delta_angle) > 1e-12) {
        delta_R = Eigen::AngleAxisd(delta_angle, delta_x.segment<3>(0).normalized());
    }
    *R = R->eval() * delta_R;

    // Update position.
    *p += delta_x.segment<3>(3);
}

struct StateBlock {
    // Global ID.
    long int id = -1;
    // Error state size.
    int size = 0;
    // Index in state vector/covariance.
    int state_idx = 0;    
};

struct WheelPose : public StateBlock {
    WheelPose() { size = 6; }

    Eigen::Matrix3d G_R_O;
    Eigen::Vector3d G_p_O;

    void Update(const Eigen::Matrix<double, 6, 1>& delta_x) {
        UpdateSE3(delta_x, &G_R_O, &G_p_O);
    }
};

struct CameraFrame : public StateBlock {
    CameraFrame() { size = 6; }

    double timestamp;

    // SE3 of this frame.
    Eigen::Matrix3d G_R_C;
    Eigen::Vector3d G_p_C;

    // Features in this frame. ID to point in image.
    std::unordered_map<long int, Eigen::Vector2d> id_pt_map;

    void Update(const Eigen::Matrix<double, 6, 1>& delta_x) {
        UpdateSE3(delta_x, &G_R_C, &G_p_C);
    }
};
using CameraFramePtr = std::shared_ptr<CameraFrame>;

struct Extrinsic : public StateBlock {
    Extrinsic() { size = 6; }

    Eigen::Matrix3d O_R_C;
    Eigen::Vector3d O_p_C;

    void Update(const Eigen::Matrix<double, 6, 1>& delta_x) {
        UpdateSE3(delta_x, &O_R_C, &O_p_C);
    }
};

struct WheelIntrinsic : public StateBlock {
    WheelIntrinsic() { size = 3; }

    double kl;
    double kr;
    double b;

    void Update(const Eigen::Vector3d& delta_x) {
        kl += delta_x[0];
        kr += delta_x[1];
        b  += delta_x[2];
    }
};

}  // namespace FilterFusion