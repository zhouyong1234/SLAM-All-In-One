#include <FilterFusion/FilterFusionSystem.h>

#include <assert.h>
#include <glog/logging.h>

#include <TGK/Camera/PinholeRanTanCamera.h>
#include <TGK/Geometry/Triangulator.h>
#include <TGK/ImageProcessor/KLTFeatureTracker.h>
#include <FilterFusion/ParamLoader.h>
#include <FilterFusion/StateAugmentor.h>
#include <FilterFusion/StateMarginalizer.h>

namespace FilterFusion {

long int FilterFusionSystem::kFrameId = -1;

FilterFusionSystem::FilterFusionSystem(const std::string& param_file) 
    : initialized_(false), odom_G_R_O_(Eigen::Matrix3d::Identity()), odom_G_p_O_(Eigen::Vector3d::Zero()) {
    /// Load parameters.
    LoadParam(param_file, &param_);

    config_.compute_raw_odom_ = param_.sys_config.compute_raw_odom;
    config_.sliding_window_size_ = param_.sys_config.sliding_window_size;
    config_.enable_plane_update = param_.sys_config.enable_plane_update;
    config_.enable_gps_update = param_.sys_config.enable_gps_update;

    /// Initialize all modules.
    data_sync_ = std::make_unique<TGK::DataSynchronizer::WheelImageSynchronizer>();

    initializer_ = std::make_unique<Initializer>(param_.extrinsic.O_R_C, param_.extrinsic.O_p_C, 
        param_.wheel_param.kl, param_.wheel_param.kr, param_.wheel_param.b);

    propagator_ = std::make_unique<Propagator>(param_.wheel_param.kl, param_.wheel_param.kr, param_.wheel_param.b, 
                                               param_.wheel_param.noise_factor);

    camera_ = std::make_shared<TGK::Camera::PinholeRadTanCamera>(
        param_.cam_intrinsic.width, param_.cam_intrinsic.height,
        param_.cam_intrinsic.fx, param_.cam_intrinsic.fy,
        param_.cam_intrinsic.cx, param_.cam_intrinsic.cy,
        param_.cam_intrinsic.k1, param_.cam_intrinsic.k2,
        param_.cam_intrinsic.p1, param_.cam_intrinsic.p2,
        param_.cam_intrinsic.k3);

    const auto triangulator = std::make_shared<TGK::Geometry::Triangulator>(param_.tri_config, camera_);

    // Create feature tracker.
    Eigen::Matrix<double, 8, 1> cam_intrin;
    cam_intrin << param_.cam_intrinsic.fx, param_.cam_intrinsic.fy,
                  param_.cam_intrinsic.cx, param_.cam_intrinsic.cy,
                  param_.cam_intrinsic.k1, param_.cam_intrinsic.k2,
                  param_.cam_intrinsic.p1, param_.cam_intrinsic.p2;

    feature_tracker_ = std::make_shared<TGK::ImageProcessor::KLTFeatureTracker>(param_.tracker_config);
    sim_feature_tracker_ = std::make_shared<TGK::ImageProcessor::SimFeatureTrakcer>();  

    visual_updater_ = std::make_unique<VisualUpdater>(param_.visual_updater_config, camera_, feature_tracker_, triangulator);
    
    if (config_.enable_plane_update) {
        plane_updater_ = std::make_unique<PlaneUpdater>(param_.plane_updater_config);
    } 

    if (config_.enable_gps_update) {
        gps_updater_ = std::make_unique<GpsUpdater>(param_.extrinsic.C_p_Gps);
    }

    viz_ = std::make_unique<Visualizer>(param_.viz_config);

    if (config_.compute_raw_odom_) {
        wheel_propagator_ = std::make_unique<TGK::WheelProcessor::WheelPropagator>(
            param_.wheel_param.kl, param_.wheel_param.kr, param_.wheel_param.b);
    }
}

bool FilterFusionSystem::FeedWheelData(const double timestamp, const double left, const double right) {
    // Convert to internal struct.
    const TGK::BaseType::WheelDataPtr wheel_ptr = std::make_shared<TGK::BaseType::WheelData>();
    wheel_ptr->timestamp = timestamp;
    wheel_ptr->left = left;
    wheel_ptr->right = right;

    // Sync with image data.
    std::vector<TGK::BaseType::WheelDataConstPtr> wheel_data_segment;
    TGK::BaseType::MonoImageDataConstPtr img_ptr;
    if (!data_sync_->FeedWheelData(wheel_ptr, &wheel_data_segment, &img_ptr)) { 
        return true;
    }

    // Initialize.
    if (!initialized_) {
        initializer_->Initialize(img_ptr->timestamp, &state_);

        // Adjust initialization for GPS Updater.
        if (config_.enable_gps_update) {
            if (latest_gps_data_ == nullptr || 
                std::abs(latest_gps_data_->timestamp - img_ptr->timestamp) > 0.5) {
                return true;
            }

            // Use bigger covariance.
            state_.covariance.block<3, 3>(3, 3) = latest_gps_data_->cov;
            state_.covariance(2, 2) = 180. * 180. * kDeg2Rad * kDeg2Rad;

            // Set init lon lat heig for gps updater.
            gps_updater_->SetInitLonLatHei(latest_gps_data_->lon_lat_hei);
        } // For gps initialization.

        initialized_ = true;
        return true;
    }

    // Propagate state.
    for (size_t i = 1; i < wheel_data_segment.size(); ++i) {
        const auto begin_wheel = wheel_data_segment[i-1];
        const auto end_wheel = wheel_data_segment[i];

        // Check timestamp.
        assert(std::abs(begin_wheel->timestamp - state_.timestamp) < 1e-6);

        propagator_->Propagate(begin_wheel->left, begin_wheel->right,
                               end_wheel->left, end_wheel->right,
                               &state_);

        // Set timestamp.
        state_.timestamp = end_wheel->timestamp;
        
        // Compute raw wheel odometry for comparison with FilterFusion.
        if (wheel_propagator_  != nullptr) {
            wheel_propagator_->PropagateUsingEncoder(begin_wheel->left, begin_wheel->right, 
                                                     end_wheel->left, end_wheel->right, 
                                                     &odom_G_R_O_, &odom_G_p_O_);
        }
    }

    /// 1. Visual update
    // Augment state / Clone new camera state.
    AugmentState(img_ptr->timestamp, (++kFrameId), &state_);

    // Track features.
    // 光流跟踪
    const auto image_type = img_ptr->type;
    std::vector<Eigen::Vector2d> tracked_pts; 
    std::vector<long int> tracked_pt_ids;
    std::vector<long int> lost_pt_ids;
    std::set<long int> new_pt_ids;
    if (image_type == TGK::BaseType::MeasureType::kMonoImage) {
        feature_tracker_->TrackImage(img_ptr->image, &tracked_pts, &tracked_pt_ids, &lost_pt_ids, &new_pt_ids);
    } else if (image_type == TGK::BaseType::MeasureType::kSimMonoImage) {
        const TGK::BaseType::SimMonoImageDataConstPtr sim_img_ptr 
            = std::dynamic_pointer_cast<const TGK::BaseType::SimMonoImageData>(img_ptr);
        sim_feature_tracker_->TrackSimFrame(sim_img_ptr->features, sim_img_ptr->feature_ids, 
                                            &tracked_pts, &tracked_pt_ids, &lost_pt_ids, &new_pt_ids);
    } else {
        LOG(ERROR) << "Not surpport image type.";
        exit(EXIT_FAILURE);
    }

    // Do not marginalize the last state if no enough camera state in the buffer.
    const bool marg_old_state = state_.camera_frames.size() >= config_.sliding_window_size_;

    // Update state.
    std::vector<Eigen::Vector2d> tracked_features;
    std::vector<Eigen::Vector2d> new_features;
    std::vector<Eigen::Vector3d> map_points;
    visual_updater_->UpdateState(img_ptr->image, marg_old_state, 
                                 tracked_pts, tracked_pt_ids, lost_pt_ids, new_pt_ids,
                                 &state_, &tracked_features, &new_features, &map_points);

    // Marginalize old state.
    if (marg_old_state) { MargOldestState(&state_); }

    /// 2. Plane Update.
    if (config_.enable_plane_update && plane_updater_ != nullptr) {
        plane_updater_->UpdateState(&state_);
    } 

    /// Visualize.
    viz_->DrawWheelPose(state_.wheel_pose.G_R_O, state_.wheel_pose.G_p_O);
    viz_->DrawFeatures(map_points);
    viz_->DrawCameras(GetCameraPoses());

    if (image_type == TGK::BaseType::MeasureType::kMonoImage) {
        viz_->DrawImage(img_ptr->image, tracked_features, new_features);
    } else if (image_type == TGK::BaseType::MeasureType::kSimMonoImage) {
        viz_->DrawColorImage(img_ptr->image);
    }

    // Draw raw wheel odometry.
    if (config_.compute_raw_odom_) {
        viz_->DrawWheelOdom(odom_G_R_O_, odom_G_p_O_);
    }

    return true;
}
    
bool FilterFusionSystem::FeedImageData(const double timestamp, const cv::Mat& image) {
    // Convert to internal struct.
    const TGK::BaseType::MonoImageDataPtr img_ptr = std::make_shared<TGK::BaseType::MonoImageData>();
    img_ptr->timestamp = timestamp;
    img_ptr->image = image;

    // Sync with wheel data.
    data_sync_->FeedMonoImageData(img_ptr);
    return true;
}

bool FilterFusionSystem::FeedSimData(const double timestamp, const cv::Mat& image, 
                            const std::vector<Eigen::Vector2d>& features,
                            const std::vector<long int>& feature_ids) {
    // Convert to internal struct.
    const TGK::BaseType::SimMonoImageDataPtr sim_img_ptr = std::make_shared<TGK::BaseType::SimMonoImageData>();
    sim_img_ptr->timestamp = timestamp;
    sim_img_ptr->features = features;
    sim_img_ptr->feature_ids = feature_ids;
    sim_img_ptr->image = image;

    // Sync with wheel data.
    data_sync_->FeedMonoImageData(sim_img_ptr);
    return true;
}

bool FilterFusionSystem::FeedGpsData(const double timestamp, const double longitude, const double latitude, const double height,
                 const Eigen::Matrix3d& cov) {
    if (!config_.enable_gps_update) {
        LOG(ERROR) << "[FeedGpsData]: Gps update not enabled.";
        return false;
    }

    const TGK::BaseType::GpsDataPtr gps_data_ptr = std::make_shared<TGK::BaseType::GpsData>();
    gps_data_ptr->timestamp = timestamp;
    gps_data_ptr->lon_lat_hei << longitude, latitude, height;
    gps_data_ptr->cov = cov;

    // Draw raw gps data.
    Eigen::Vector3d init_llh;
    if (gps_updater_->GetInitLonLatHei(&init_llh)) {
        Eigen::Vector3d G_p_Gps = ConvertLonLatHeiToENU(init_llh, gps_data_ptr->lon_lat_hei);

        // TODO: Do not use height.
        G_p_Gps[2] = 0.;

        viz_->DrawGps(G_p_Gps);
    }

    if (!initialized_) {
        latest_gps_data_ = gps_data_ptr;
        LOG(WARNING) << "[FeedGpsData]: System not initialized.";
        return false;
    }

    gps_updater_->UpdateState(gps_data_ptr, &state_);

    return true;
}

void FilterFusionSystem::FeedGroundTruth(const double timestamp, const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O) {
    viz_->DrawGroundTruth(G_R_O, G_p_O);
}

std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> FilterFusionSystem::GetCameraPoses() {
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> cam_poses;
    cam_poses.clear();
    for (const auto& cam_fm : state_.camera_frames) {
        cam_poses.emplace_back(cam_fm->G_R_C, cam_fm->G_p_C);
    }

    return cam_poses;
}

}  // namespace FilterFusion