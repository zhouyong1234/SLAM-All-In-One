#pragma once 

#include <memory>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include <TGK/Camera/Camera.h>
#include <TGK/DataSynchronizer/WheelImageSynchronizer.h>
#include <TGK/Geometry/Triangulator.h>
#include <TGK/ImageProcessor/FeatureTracker.h>
#include <TGK/ImageProcessor/SimFeatureTracker.h>

#include <FilterFusion/Initializer.h>
#include <FilterFusion/Parameter.h>
#include <FilterFusion/Propagator.h>
#include <FilterFusion/State.h>
#include <FilterFusion/VisualUpdater.h>
#include <FilterFusion/PlaneUpdater.h>
#include <FilterFusion/GpsUpdater.h>
#include <FilterFusion/Visualizer.h>

namespace FilterFusion {

class FilterFusionSystem {
public:
    struct Config {
        int sliding_window_size_ = 10;
        bool compute_raw_odom_ =  true;

        bool enable_plane_update = true;
        bool enable_gps_update = true;
    };

    FilterFusionSystem(const std::string& param_file);

    bool FeedWheelData(const double timestamp, const double left, const double right);

    bool FeedImageData(const double timestamp, const cv::Mat& image);

    bool FeedGpsData(const double timestamp, const double longitude, const double latitude, const double height,
                     const Eigen::Matrix3d& cov);

    bool FeedSimData(const double timestamp, const cv::Mat& image, 
                     const std::vector<Eigen::Vector2d>& features,
                     const std::vector<long int>& feature_ids);
                     
    void FeedGroundTruth(const double timestamp, const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O);

private:
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> GetCameraPoses();

    Config config_;
    std::shared_ptr<TGK::Camera::Camera> camera_;
    std::unique_ptr<TGK::DataSynchronizer::WheelImageSynchronizer> data_sync_;
    std::unique_ptr<Initializer> initializer_;
    std::unique_ptr<Visualizer> viz_;
    std::unique_ptr<Propagator> propagator_;
    std::unique_ptr<VisualUpdater> visual_updater_;
    std::unique_ptr<PlaneUpdater> plane_updater_ = nullptr;
    std::unique_ptr<GpsUpdater> gps_updater_ = nullptr;
    std::shared_ptr<TGK::ImageProcessor::FeatureTracker> feature_tracker_;
    std::shared_ptr<TGK::ImageProcessor::SimFeatureTrakcer> sim_feature_tracker_; 

    bool initialized_;
    Parameter param_;

    State state_;

    static long int kFrameId;

    // Raw wheel Odometry, just for comparison.
    Eigen::Matrix3d odom_G_R_O_;
    Eigen::Vector3d odom_G_p_O_;
    std::unique_ptr<TGK::WheelProcessor::WheelPropagator> wheel_propagator_;

    // JUST FOR INITIALIZATION.
    TGK::BaseType::GpsDataConstPtr latest_gps_data_ = nullptr;
};

}  // namespace FilterFusion