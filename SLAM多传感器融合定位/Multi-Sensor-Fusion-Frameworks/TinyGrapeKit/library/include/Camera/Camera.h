#pragma once

#include <memory>

#include <Eigen/Core>

namespace TGK {
namespace Camera {

class Camera {
public:
    Camera(const int width, const int height) : width_(width), height_(height) { }
    
    virtual bool CameraToImage(const Eigen::Vector3d& C_p, Eigen::Vector2d* I_p, 
                               Eigen::Matrix<double, 2, 3>* J_Ip_wrt_Cp = nullptr) const = 0;

    virtual bool NSPToImage(const Eigen::Vector2d& NSP_p, Eigen::Vector2d* I_p, 
                            Eigen::Matrix2d* J_Ip_wrt_NSP = nullptr) const = 0;

    virtual Eigen::Vector3d ImageToCamera(const Eigen::Vector2d& I_p, const double z = 1.) const = 0;

    inline bool InImage(const int u, const int v, const int u_edge = 0, const int v_edge = 0) const {
        if (u < u_edge || v < v_edge || u >= (width_ - u_edge) || v >= (height_ - v_edge)) {
            return false;
        }
        return true;
    }

    inline bool InImage(const Eigen::Vector2d& I_p) const {
        return InImage(I_p[0], I_p[1]);
    }

    inline int width() { return width_; }
    inline int height() { return height_; }
    
protected: 
    int width_;
    int height_;
};

using CameraPtr = std::shared_ptr<Camera>;

}  // namespace Camera
}  // namespace TGK