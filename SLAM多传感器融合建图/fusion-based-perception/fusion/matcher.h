// Copyright 2021 Sui Fang

#pragma once

//#include <image_geometry/pinhole_camera_model.h>
#include <string>
#include "utils.h"
#include "state.h"
#include "yaml-cpp/yaml.h"
// #include "hungarian.h"
#include "munkres.hpp"

namespace kit {
namespace perception {
namespace fusion {

class Matcher {
 public:
    explicit Matcher(const std::string& config_file);

    // void Setcamera_intrinsic(
    //     const image_geometry::PinholeCameraModel& cam_model) {
    //     cam_intrinsic_(0) = cam_model.fx();
    //     cam_intrinsic_(1) = cam_model.fy();
    //     cam_intrinsic_(2) = cam_model.Tx();
    //    cam_intrinsic_(3) = cam_model.Ty();
    //    cam_intrinsic_(4) = cam_model.cx();
    //    cam_intrinsic_(5) = cam_model.cy();
    //    initialized_ = true;
    //}

    void SetMatcherReady() { initialized_ = true; }
    bool IsMatcherReady() { return initialized_; }

    bool Match(const RadarObjectListPtr& radar_obj_list,
               const FusionObjectListPtr& res_obj_list,
               std::map<size_t, int>& radar_global_map);

    bool Match(const CameraObjectListPtr& cam_obj_list,
               const FusionObjectListPtr& res_obj_list,
               std::map<size_t, int>& cam_global_map);

    bool Match(const LiDARObjectListPtr& lidar_obj_list,
               const FusionObjectListPtr& res_obj_list,
               std::map<size_t, int>& lidar_global_map);

    void Match(const RadarObjectListPtr& radar_obj_list,
                    const CameraObjectListPtr& cam_obj_list,
                    const FusionObjectListPtr& res_obj_list);

 private:
    float IoUCamToFusion(const CameraObjectPtr& cam_obj,
            const FusionObjectPtr& fusion_obj);
    float IoULiDARToFusion(const LiDARObjectPtr& lidar_obj,
            const FusionObjectPtr& fusion_obj);
    float IoURadarToFusion(const RadarObjectPtr& radar_obj,
            const FusionObjectPtr& fusion_obj);

    void Preprocess(const RadarObjectListPtr& radar_obj_list,
                    const FusionObjectListPtr& fusion_obj_list);

    Eigen::Vector3d TransformToCameraFrame(const Eigen::Vector3d& point,
                                           const Eigen::Affine3d& extrinsic);

    Eigen::Vector2d TransformToImageFrame(const Eigen::Vector3d& point,
                                          const Vector6d& intrinsic);

    Eigen::Vector2d SizeOfRadarObjectInImage(const RadarObjectPtr& radar_obj,
                                             const Eigen::Affine3d& extrinsic,
                                             const Vector6d& intrinsic);

    bool initialized_ = false;
    Vector6d cam_intrinsic_;
    Eigen::Affine3d extrinsic_radar_to_lidar_;
    Eigen::Affine3d extrinsic_radar_to_camera_;
    Eigen::Affine3d extrinsic_radar_to_baselink_;
    Eigen::Affine3d extrinsic_lidar_to_baselink_;
    Eigen::Affine3d extrinsic_camera_to_baselink_;
    Eigen::Affine3d extrinsic_camera_to_lidar_;

    float radar_nearest_dist_thres_ = 0.3;
    float radar_furthest_dist_thres_ = 20.0;
    float default_radar_obj_length_ = 1.0;
    float default_radar_obj_width_ = 1.0;
    float default_radar_obj_height_ = 1.8;
    float matching_thres_ = 0.1;

    int matching_offset_in_ux_ = 0;
    int matching_offset_in_vy_ = 50;

    SensorType main_sensor_ = SensorType::LIDAR;

    // std::shared_ptr<Hungarian<int>> hungarian_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace kit
