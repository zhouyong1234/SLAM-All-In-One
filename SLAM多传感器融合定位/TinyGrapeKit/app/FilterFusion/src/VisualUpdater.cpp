#include <FilterFusion/VisualUpdater.h>

#include <vector>
#include <unordered_set>

#include <glog/logging.h>

#include <TGK/Util/Util.h>
#include <FilterFusion/UpdaterUtil.h>

namespace FilterFusion {

VisualUpdater::VisualUpdater(const Config& config,
                             const std::shared_ptr<TGK::Camera::Camera> camera,
                             const std::shared_ptr<TGK::ImageProcessor::FeatureTracker> feature_tracker,
                             const std::shared_ptr<TGK::Geometry::Triangulator> triangulator)
    : config_(config), camera_(camera), feature_tracker_(feature_tracker), triangulator_(triangulator) { }

void VisualUpdater::UpdateState(const cv::Mat& image, const bool marg_oldest, 
                                const std::vector<Eigen::Vector2d>& tracked_pts, 
                                const std::vector<long int>& tracked_pt_ids,
                                const std::vector<long int>& lost_pt_ids,
                                const std::set<long int>& new_pt_ids,
                                State* state, 
                                std::vector<Eigen::Vector2d>* tracked_features,
                                std::vector<Eigen::Vector2d>* new_features,
                                std::vector<Eigen::Vector3d>* map_points) {
    // Collect features for observation.
    *tracked_features = tracked_pts; 
    for (size_t i = 0; i < tracked_pt_ids.size(); ++i) {
        const long long pt_id = tracked_pt_ids[i];
        if (new_pt_ids.count(pt_id) > 0) {
            new_features->push_back(tracked_pts[i]);
        }
    }

    // Assuming a new clone has been inserted into the sliding window.
    // 相机位姿通过里程计位姿得到
    CameraFramePtr new_cam_frame = state->camera_frames.back();

    // Insert tracked features to frame.
    for (size_t i = 0; i < tracked_pt_ids.size(); ++i) {
        const long int ft_id = tracked_pt_ids[i];
        const Eigen::Vector2d& pt = tracked_pts[i];
        // Insert feature to camera frame.
        new_cam_frame->id_pt_map[ft_id] = pt;
    }

    ///  Collect features use to update.
    // 1. Tracked lost features.
    std::unordered_set<long int> lost_ft_ids_set(lost_pt_ids.begin(), lost_pt_ids.end());
    // 2. We always marginalize the last camera frame. So we need to collect all features in the last frame.
    if (marg_oldest) {
        const CameraFramePtr marg_cam = state->camera_frames.front();
        for (const auto& id_pt : marg_cam->id_pt_map) {
            lost_ft_ids_set.insert(id_pt.first);
        }
    }

    // Collect image point & camera state pairs.
    std::vector<std::vector<std::pair<Eigen::Vector2d, CameraFramePtr>>> features_obs;
    features_obs.reserve(lost_ft_ids_set.size());
    for (const long int id : lost_ft_ids_set) {
        std::vector<std::pair<Eigen::Vector2d, CameraFramePtr>> one_feature;
        for (const auto cam_fm : state->camera_frames) {
            const auto iter = cam_fm->id_pt_map.find(id);
            if (iter == cam_fm->id_pt_map.end()) { continue; }
            const Eigen::Vector2d& im_pt = iter->second;
            one_feature.emplace_back(im_pt, cam_fm);
        }

        if (one_feature.size() < 3) { continue; }
        features_obs.push_back(one_feature);
    }
    const int state_size = state->covariance.rows();

    /// Triangulate points.
    map_points->clear();
    map_points->reserve(features_obs.size());
    std::vector<FeatureObservation> features_full_obs;
    features_full_obs.reserve(features_obs.size());
    for (const std::vector<std::pair<Eigen::Vector2d, CameraFramePtr>>& one_ft_obs : features_obs) {
        FeatureObservation one_feaute;
        one_feaute.im_pts.reserve(one_ft_obs.size());
        one_feaute.camera_frame.reserve(one_ft_obs.size());
        std::vector<Eigen::Matrix3d> G_R_Cs;
        std::vector<Eigen::Vector3d> G_p_Cs;
        for (const auto& one_obs : one_ft_obs) {
            G_R_Cs.push_back(one_obs.second->G_R_C);
            G_p_Cs.push_back(one_obs.second->G_p_C);
            one_feaute.im_pts.push_back(one_obs.first);
            one_feaute.camera_frame.push_back(one_obs.second);
        }

        // Check camera distance.
        if ((one_feaute.camera_frame.front()->G_p_C - one_feaute.camera_frame.back()->G_p_C).norm() < 
            config_.min_cam_dist_to_triangulate) {
            continue;
        }

        // 三角化得到地图点
        if (!triangulator_->Triangulate(G_R_Cs, G_p_Cs, one_feaute.im_pts, &one_feaute.G_p)) { continue; }
        features_full_obs.push_back(one_feaute);
        map_points->push_back(one_feaute.G_p);
    }

    // Compute Jacobian.
    Eigen::MatrixXd H;
    Eigen::VectorXd r;
    ComputeVisualResidualJacobian(features_full_obs, state_size, &r, &H);

    const double window_length = (state->camera_frames.front()->G_p_C  - state->camera_frames.back()->G_p_C).norm();
    if (H.rows() > config_.min_res_size && window_length > config_.min_window_length) { 
        // Compress measurement.
        Eigen::MatrixXd H_cmp;
        Eigen::VectorXd r_cmp;
        // QR分解压缩矩阵维度
        CompressMeasurement(H, r, &H_cmp, &r_cmp);

        /// EKF update.
        Eigen::MatrixXd V(H_cmp.rows(), H_cmp.rows());
        V.setIdentity();
        V = V.eval() * config_.visual_noise;
        // 状态更新
        EKFUpdate(H_cmp, r_cmp, V, state);
    }

    // Remove use/lost features.
    RemoveUsedFeatures(lost_ft_ids_set, state);
}

void VisualUpdater::RemoveUsedFeatures(const std::unordered_set<long int>& lost_ft_ids_set, State* state) {
    // Remove used features
    for (const auto& id : lost_ft_ids_set) {
        // Remove features in state.
        for (auto cam_fm : state->camera_frames) {
            auto iter = cam_fm->id_pt_map.find(id);
            if (iter == cam_fm->id_pt_map.end()) { continue; }
            cam_fm->id_pt_map.erase(iter);
        }

        // Remove features in feature tracker.
        feature_tracker_->DeleteFeature(id);
    }
}

bool VisualUpdater::ComputeProjectionResidualJacobian(const Eigen::Matrix3d& G_R_C,
                                                const Eigen::Vector3d& G_p_C,
                                                const Eigen::Vector2d& im_pt,
                                                const Eigen::Vector3d& G_p,
                                                Eigen::Vector2d* res,
                                                Eigen::Matrix<double, 2, 6>* J_wrt_cam_pose,
                                                Eigen::Matrix<double, 2, 3>* J_wrt_Gp) {
    const Eigen::Vector3d C_p = G_R_C.transpose() * (G_p - G_p_C);
    Eigen::Vector2d exp_im_pt; 
    Eigen::Matrix<double, 2, 3> J_wrt_Cp;
    if (!camera_->CameraToImage(C_p, &exp_im_pt, &J_wrt_Cp)) {
        return false;
    }
    
    // Compute residual
    *res = im_pt - exp_im_pt;

    // Jacobian.
    Eigen::Matrix<double, 3, 6> J_Cp_wrt_cam_pose;
    J_Cp_wrt_cam_pose << TGK::Util::Skew(C_p), -G_R_C.transpose();

    const Eigen::Matrix3d J_Cp_wrt_Gp = G_R_C.transpose();
    *J_wrt_cam_pose = J_wrt_Cp * J_Cp_wrt_cam_pose;
    *J_wrt_Gp = J_wrt_Cp * J_Cp_wrt_Gp;

    return true;
}


// 计算一个点的状态和特征点的雅克比
void VisualUpdater::ComputeOnePointResidualJacobian(const FeatureObservation& one_feature_obs, 
                                              const int state_size,
                                              Eigen::VectorXd* residual, 
                                              Eigen::MatrixXd* Hx,
                                              Eigen::Matrix<double, Eigen::Dynamic, 3>* Hf) {
    // 3D点
    const Eigen::Vector3d& G_p = one_feature_obs.G_p;
    // 相机位姿
    const std::vector<CameraFramePtr>& camera_frame = one_feature_obs.camera_frame;
    // 2D点
    const std::vector<Eigen::Vector2d>& im_pts = one_feature_obs.im_pts;

    std::vector<Eigen::Vector2d> residuals;
    // 状态雅克比
    std::vector<Eigen::Matrix<double, 2, Eigen::Dynamic>> Js_wrt_state;
    // 特征点雅克比
    std::vector<Eigen::Matrix<double, 2, 3>> Js_wrt_feature;
    for (size_t i = 0; i < camera_frame.size(); ++i) {
        int col_idx = camera_frame[i]->state_idx;
        const Eigen::Matrix3d& G_R_C = camera_frame[i]->G_R_C;
        const Eigen::Vector3d& G_p_C = camera_frame[i]->G_p_C;
        const Eigen::Vector2d& im_pt = im_pts[i];
        
        Eigen::Vector2d res;
        Eigen::Matrix<double, 2, 6> J_wrt_cam_pose;
        Eigen::Matrix<double, 2, 3> J_wrt_Gp;
        if (!ComputeProjectionResidualJacobian(G_R_C, G_p_C, im_pt, G_p, &res, &J_wrt_cam_pose, &J_wrt_Gp)) {
            continue;
        }

        residuals.push_back(res);

        Eigen::Matrix<double, 2, Eigen::Dynamic> one_J_wrt_state(2, state_size);
        one_J_wrt_state.setZero();
        one_J_wrt_state.block<2, 6>(0, col_idx) = J_wrt_cam_pose;
        Js_wrt_state.push_back(one_J_wrt_state);
        Js_wrt_feature.push_back(J_wrt_Gp);
    }

    const int num_rows = Js_wrt_state.size() * 2;
    residual->resize(num_rows);
    Hx->resize(num_rows, state_size);
    Hf->resize(num_rows, 3);

    for (size_t i = 0; i < Js_wrt_state.size(); ++i) {
        const int row_idx = 2 * i;
        residual->segment<2>(row_idx) = residuals[i];
        Hx->block(row_idx, 0, 2, state_size) = Js_wrt_state[i];
        Hf->block(row_idx, 0, 2, 3) = Js_wrt_feature[i];
    }
}


void VisualUpdater::ComputeVisualResidualJacobian(const std::vector<FeatureObservation>& features_full_obs, 
                                            const int state_size,
                                            Eigen::VectorXd* res, 
                                            Eigen::MatrixXd* Jacobian) {
    std::vector<Eigen::VectorXd> r_bars;
    std::vector<Eigen::MatrixXd> H_bars;
    r_bars.reserve(features_full_obs.size());
    H_bars.reserve(features_full_obs.size());
    int num_rows = 0;
    for (const FeatureObservation& one_feature_obs : features_full_obs) {
        Eigen::VectorXd res;
        Eigen::MatrixXd Hx;
        Eigen::Matrix<double, Eigen::Dynamic, 3> Hf;
        ComputeOnePointResidualJacobian(one_feature_obs, state_size, &res, &Hx, &Hf);
        if (res.size() < 4) { continue; }

        // Left projection.
        Eigen::MatrixXd H_bar;
        Eigen::VectorXd r_bar;
        // 左零空间投影
        LeftNullspaceProjection(Hx, Hf, res, &H_bar, &r_bar);

        // 多个特征点叠加
        H_bars.push_back(H_bar);
        r_bars.push_back(r_bar);
        num_rows += H_bar.rows();
    }

    // Stack to big H and r matrices. 
    res->resize(num_rows);
    Jacobian->resize(num_rows, state_size);
    int row_idx = 0;
    for (size_t i = 0; i < H_bars.size(); ++i) {
        const int one_num_rows = H_bars[i].rows();
        res->segment(row_idx, one_num_rows) = r_bars[i];
        Jacobian->block(row_idx, 0, one_num_rows, state_size) = H_bars[i];
        row_idx += one_num_rows;
    }
}

}  // namespace FilterFusion