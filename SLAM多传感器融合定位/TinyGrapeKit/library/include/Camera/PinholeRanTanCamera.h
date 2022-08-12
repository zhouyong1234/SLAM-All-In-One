#pragma once

#include "Camera/Camera.h"

namespace TGK {
namespace Camera {

class PinholeRadTanCamera : public Camera {
public:
    PinholeRadTanCamera(const int width, const int height,
                        const double fx, const double fy,
                        const double cx, const double cy,
                        const double k1 = 0., const double k2 = 0.,
                        const double p1 = 0., const double p2 = 0.,
                        const double k3 = 0.);
    
    virtual bool CameraToImage(const Eigen::Vector3d& C_p, Eigen::Vector2d* I_p, 
                       Eigen::Matrix<double, 2, 3>* J_Ip_wrt_Cp = nullptr) const;
    
    virtual bool NSPToImage(const Eigen::Vector2d& NSP_p, Eigen::Vector2d* I_p, 
                            Eigen::Matrix2d* J_Ip_wrt_NSP = nullptr) const;

    virtual Eigen::Vector3d ImageToCamera(const Eigen::Vector2d& I_p, const double z = 1.) const;
    
private:
    // Project 3D points in the camera frame to the 2D image.
    Eigen::Vector2d CameraToNormalized(const Eigen::Vector3d& C_p, 
                                       Eigen::Matrix<double, 2, 3>* J_norm_wrt_Cp = nullptr) const;
    Eigen::Vector2d NormalizedToDistortion(const Eigen::Vector2d& N_p,
                                           Eigen::Matrix2d* J_dist_wrt_norm = nullptr) const;
    Eigen::Vector2d DistortionToImage(const Eigen::Vector2d& D_p,
                                      Eigen::Matrix2d* J_Ip_wrt_dist = nullptr) const;

    // Project the 2D image points to the 3D points in the camera.
    Eigen::Vector2d ImageToDistortion(const Eigen::Vector2d& I_p) const;
    Eigen::Vector2d DistortionToNormalized(const Eigen::Vector2d& D_p) const;
    Eigen::Vector3d NormalizedToCamera(const Eigen::Vector2d& N_p, const double z = 1.) const;

    // The intrinsic.
    double fx_;
    double fy_;
    double cx_;
    double cy_;

    // The distortion parameter
    double k1_;
    double k2_;
    double k3_;
    double p1_;
    double p2_;
};

}  // namespace Camera
}  // namespace TGK