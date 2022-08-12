#pragma once

#include <memory>
#include <unordered_set>

#include <opencv2/opencv.hpp>

#include <TGK/Camera/Camera.h>
#include <TGK/Geometry/Triangulator.h>
#include <TGK/ImageProcessor/FeatureTracker.h>
#include <FilterFusion/State.h>

namespace FilterFusion {

class VisualUpdater {
public:
    struct Config {
        double visual_noise = 1. * 1.; // 1 pixel std.
        double min_window_length = 3.;
        int min_res_size = 10;
        double min_cam_dist_to_triangulate = 0.2;

        double plane_rot_noise = 0.01;
        double plane_trans_noise = 0.01;
    };

    struct FeatureObservation {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::vector<CameraFramePtr> camera_frame;
        std::vector<Eigen::Vector2d> im_pts;
        Eigen::Vector3d G_p;
    };

    VisualUpdater(const Config& config,
                  const std::shared_ptr<TGK::Camera::Camera> camera,
                  const std::shared_ptr<TGK::ImageProcessor::FeatureTracker> feature_tracker,
                  const std::shared_ptr<TGK::Geometry::Triangulator> triangulator);

    void UpdateState(const cv::Mat& image, const bool marg_oldest, 
                     const std::vector<Eigen::Vector2d>& tracked_pts, 
                     const std::vector<long int>& tracked_pt_ids,
                     const std::vector<long int>& lost_pt_ids,
                     const std::set<long int>& new_pt_ids,
                     State* state, 
                     std::vector<Eigen::Vector2d>* all_features,
                     std::vector<Eigen::Vector2d>* new_features,
                     std::vector<Eigen::Vector3d>* map_points);

private:
    bool ComputeProjectionResidualJacobian(const Eigen::Matrix3d& G_R_C,
                                           const Eigen::Vector3d& G_p_C,
                                           const Eigen::Vector2d& im_pt,
                                           const Eigen::Vector3d& G_p,
                                           Eigen::Vector2d* res,
                                           Eigen::Matrix<double, 2, 6>* J_wrt_cam_pose,
                                           Eigen::Matrix<double, 2, 3>* J_wrt_Gp);

    void ComputeOnePointResidualJacobian(const FeatureObservation& one_feature_obs, 
                                         const int state_size,
                                         Eigen::VectorXd* residual, 
                                         Eigen::MatrixXd* Hx,
                                         Eigen::Matrix<double, Eigen::Dynamic, 3>* Hf);

    void ComputeVisualResidualJacobian(const std::vector<FeatureObservation>& features_full_obs, 
                                       const int state_size,
                                       Eigen::VectorXd* res, 
                                       Eigen::MatrixXd* Jacobian);

    void RemoveUsedFeatures(const std::unordered_set<long int>& lost_ft_ids_set, State* state);

    const Config config_;
    const TGK::Camera::CameraPtr camera_;
    const std::shared_ptr<TGK::ImageProcessor::FeatureTracker> feature_tracker_; 
    const std::shared_ptr<TGK::Geometry::Triangulator> triangulator_;
};

}  // namespace FilterFusion