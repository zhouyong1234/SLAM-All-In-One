#pragma once

#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <Camera/Camera.h>

namespace TGK {
namespace Simulation {

class VisualWheelCircleSim {
public:
    VisualWheelCircleSim(const std::shared_ptr<Camera::Camera> camera,
                         const double kl = 1., const double kr = 1., const double b = 1.5, 
                         const Eigen::Matrix3d& O_R_C = Eigen::Matrix3d::Identity(), 
                         const Eigen::Vector3d& O_p_C = Eigen::Vector3d::Zero(),
                         const double circle_radius = 200., const int sample_num = 5000,
                         const double view_range = 50.);

    bool SimOneFrame(double* timestamp, 
                     Eigen::Matrix3d* G_R_O,
                     Eigen::Vector3d* G_p_O,
                     double* left_wheel, double* right_wheel, 
                     std::vector<Eigen::Vector2d>* features, std::vector<long int>* feature_ids,
                     cv::Mat* image = nullptr);    

    void Reset();

    void GetAllMapPoint(std::vector<Eigen::Vector3d>* map_points, std::vector<long int>* map_point_ids);

private:
    void CrateMapPoints();

    std::shared_ptr<Camera::Camera> camera_;
    const double kl_;
    const double kr_;
    const double b_;
    const Eigen::Matrix3d O_R_C_;
    const Eigen::Vector3d O_p_C_;
    const double circle_radius_;
    const int sample_num_;
    const double view_range_;
    const double sample_dth_;
    long int frame_id_;

    // All map points.
    std::vector<Eigen::Vector3d> map_points_;
    std::vector<long int> map_point_ids_;
};

}  // namespace Simulation
}  // namespace TGK