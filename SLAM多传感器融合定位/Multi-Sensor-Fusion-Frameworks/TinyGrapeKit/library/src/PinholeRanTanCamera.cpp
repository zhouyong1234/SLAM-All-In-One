#include "Camera/PinholeRanTanCamera.h"

namespace TGK {
namespace Camera {

PinholeRadTanCamera::PinholeRadTanCamera(const int width, const int height,
                                         const double fx, const double fy,
                                         const double cx, const double cy,
                                         const double k1, const double k2,
                                         const double p1, const double p2,
                                         const double k3) 
    : Camera(width, height), fx_(fx), fy_(fy), cx_(cx), cy_(cy),
      k1_(k1), k2_(k2), p1_(p1), p2_(p2), k3_(k3) { }

bool PinholeRadTanCamera::CameraToImage(const Eigen::Vector3d& C_p, Eigen::Vector2d* I_p, 
                                 Eigen::Matrix<double, 2, 3>* J_Ip_wrt_Cp) const {
    if (C_p[2] <= 0.1) { 
        return false;
    }

    if (J_Ip_wrt_Cp) {
        Eigen::Matrix<double, 2, 3> J_norm_wrt_Cp;
        const Eigen::Vector2d N_p = CameraToNormalized(C_p, &J_norm_wrt_Cp);

        Eigen::Matrix2d J_dist_wrt_norm;
        const Eigen::Vector2d D_p = NormalizedToDistortion(N_p, &J_dist_wrt_norm);

        Eigen::Matrix2d J_Ip_wrt_dist;
        *I_p = DistortionToImage(D_p, &J_Ip_wrt_dist);

        *J_Ip_wrt_Cp = J_Ip_wrt_dist * J_dist_wrt_norm * J_norm_wrt_Cp;
        
        return InImage(*I_p);
    }
    
    const Eigen::Vector2d N_p = CameraToNormalized(C_p);
    const Eigen::Vector2d D_p = NormalizedToDistortion(N_p);
    *I_p = DistortionToImage(D_p);

    return InImage(*I_p);
}

bool PinholeRadTanCamera::NSPToImage(const Eigen::Vector2d& NSP_p, Eigen::Vector2d* I_p, 
                                     Eigen::Matrix2d* J_Ip_wrt_NSP) const {

    if (J_Ip_wrt_NSP) {
        Eigen::Matrix2d J_dist_wrt_norm;
        const Eigen::Vector2d D_p = NormalizedToDistortion(NSP_p, &J_dist_wrt_norm);

        Eigen::Matrix2d J_Ip_wrt_dist;
        *I_p = DistortionToImage(D_p, &J_Ip_wrt_dist);

        *J_Ip_wrt_NSP = J_Ip_wrt_dist * J_dist_wrt_norm;

        return InImage(*I_p);
    }
    
    const Eigen::Vector2d D_p = NormalizedToDistortion(NSP_p);
    *I_p = DistortionToImage(D_p);

    return InImage(*I_p);
}

Eigen::Vector3d PinholeRadTanCamera::ImageToCamera(const Eigen::Vector2d& I_p, const double z) const {
    const Eigen::Vector2d D_p = ImageToDistortion(I_p);
    const Eigen::Vector2d N_p = DistortionToNormalized(D_p);
    return NormalizedToCamera(N_p, z);
}

Eigen::Vector2d PinholeRadTanCamera::CameraToNormalized(const Eigen::Vector3d& C_p, 
                                                        Eigen::Matrix<double, 2, 3>* J_norm_wrt_Cp) const {
    const double one_over_z = 1. / C_p[2];
    const double x = C_p[0] * one_over_z;
    const double y = C_p[1] * one_over_z;

    if (J_norm_wrt_Cp) {
        const double one_over_z2 = one_over_z * one_over_z;
        *J_norm_wrt_Cp << one_over_z, 0.,         -C_p[0] * one_over_z2,
                          0.,         one_over_z, -C_p[1] * one_over_z2;
    }
    return Eigen::Vector2d(x, y);
}

Eigen::Vector2d PinholeRadTanCamera::NormalizedToDistortion(const Eigen::Vector2d& N_p,
                                                            Eigen::Matrix2d* J_dist_wrt_norm) const {
    const double x = N_p[0];
    const double y = N_p[1];

    // Normalized to distortion.
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r2 * r4;                                                                                                                                                                           
    const double a1 = 2. * x * y;
    const double a2 = r2 + 2. * x * x;
    const double a3 = r2 + 2. * y * y;
    const double radial_factor = 1. + k1_*r2 + k2_*r4 + k3_*r6;

    const double x_dist = radial_factor * x + p1_ * a1 + p2_ * a2;
    const double y_dist = radial_factor * y + p1_ * a3 + p2_ * a1;
    
    // Compute jacobian.
    if (J_dist_wrt_norm) {
        const double J_factor = 2. * k1_ + 4 * k2_ * r2 + 6. * k3_ * r4;
        const double x2 = x * x;
        const double y2 = y * y;
        const double J_xdist_wrt_xnorm = radial_factor + x2 * J_factor + 2. * p1_ * y + 6. * p2_ * x;
        const double J_xdist_wrt_ynorm = x * y * J_factor + 2. * p1_ * x + 2. * p2_ * y;
        const double J_ydist_wrt_xnorm = J_xdist_wrt_ynorm;
        const double J_ydist_wrt_ynorm = radial_factor + y2 * J_factor + 6. * p1_ * y + 2. * p2_ * x;
        *J_dist_wrt_norm << J_xdist_wrt_xnorm, J_xdist_wrt_ynorm,
                            J_ydist_wrt_xnorm, J_ydist_wrt_ynorm;
    }

    return Eigen::Vector2d(x_dist, y_dist);
}

Eigen::Vector2d PinholeRadTanCamera::DistortionToImage(const Eigen::Vector2d& D_p,
                                                       Eigen::Matrix2d* J_Ip_wrt_dist) const {
    const double u = D_p[0] * fx_ + cx_;
    const double v = D_p[1] * fy_ + cy_;

    if (J_Ip_wrt_dist) {
        *J_Ip_wrt_dist << fx_, 0,
                          0.,  fy_;
    }

    return Eigen::Vector2d(u, v);
} 

// Project the 2D image points to the 3D points in the camera.
Eigen::Vector2d PinholeRadTanCamera::ImageToDistortion(const Eigen::Vector2d& I_p) const {
    const double x_dist = (I_p[0] - cx_) / fx_;
    const double y_dist = (I_p[1] - cy_) / fy_;
    return Eigen::Vector2d(x_dist, y_dist);
}

Eigen::Vector2d PinholeRadTanCamera::DistortionToNormalized(const Eigen::Vector2d& D_p) const {
    const int max_niter = 10;
    const double epsilon = 1e-12;

    const double x_dist = D_p[0];
    const double y_dist = D_p[1];
    double x = x_dist;
    double y = y_dist;

    int cnt = 0;
    double error = std::numeric_limits<double>::max();
    while ((cnt++) < max_niter && error > epsilon) {
        // Update x, y.
        const double r2 = x * x + y * y;
        const double icdist = 1. / (1. + ((k3_ * r2 + k2_) * r2 + k1_) * r2);
        const double delta_x = 2. * p1_ * x * y + p2_ * (r2 + 2. * x * x);
        const double delta_y = p1_ * (r2 + 2. * y * y) + 2. * p2_ * x * y;
        x = (x_dist - delta_x) * icdist;
        y = (y_dist - delta_y) * icdist;    

        // Compute error.
        const Eigen::Vector2d D_p_predict = NormalizedToDistortion(Eigen::Vector2d(x, y));
        error = (D_p - D_p_predict).norm();
    }
    
    return Eigen::Vector2d(x, y);
}

Eigen::Vector3d PinholeRadTanCamera::NormalizedToCamera(const Eigen::Vector2d& N_p, const double z) const {
    const double x = N_p[0] * z;
    const double y = N_p[1] * z;
    return Eigen::Vector3d(x, y, z);
}

}  // namespace Camera
}  // namespace TGK