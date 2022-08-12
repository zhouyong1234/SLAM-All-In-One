#pragma once

#include <Eigen/Core>

#include <FilterFusion/Visualizer.h>
#include <FilterFusion/VisualUpdater.h>
#include "FilterFusion/PlaneUpdater.h"
#include <TGK/Geometry/Triangulator.h>
#include <TGK/ImageProcessor/KLTFeatureTracker.h>

namespace FilterFusion {

struct CamParam {
    double fx;
    double s;
    double fy;
    double cx;
    double cy;
    double k1;
    double k2;
    double p1;
    double p2;
    double k3;

    int width;
    int height;
};

struct WheelParam {
    double kl;
    double kr;
    double b;
    
    double noise_factor;
};

struct ExtrinsicParam {
    Eigen::Matrix3d O_R_C;
    Eigen::Vector3d O_p_C;

    Eigen::Vector3d C_p_Gps;
};

struct SysConfig {
    int sliding_window_size;
    bool compute_raw_odom;
    bool enable_plane_update;
    bool enable_gps_update;
};

struct Parameter {
    // Camera instrinc.
    CamParam cam_intrinsic;
    // Wheel Intrinsic.
    WheelParam wheel_param;
    // Extrinsic.
    ExtrinsicParam extrinsic;

    Visualizer::Config viz_config;
    TGK::Geometry::Triangulator::Config tri_config;
    TGK::ImageProcessor::KLTFeatureTracker::Config tracker_config;
    VisualUpdater::Config visual_updater_config;
    PlaneUpdater::Config plane_updater_config;

    SysConfig sys_config;
};

}  // namespace FilterFusion