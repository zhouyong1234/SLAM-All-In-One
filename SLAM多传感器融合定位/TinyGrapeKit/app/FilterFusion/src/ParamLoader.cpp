#include <FilterFusion/ParamLoader.h>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

namespace FilterFusion {

void LoadParam(const std::string& param_file, Parameter* params) {
    cv::FileStorage cv_params(param_file, cv::FileStorage::READ);
    
    // Load camera param.
    params->cam_intrinsic.fx = cv_params["Camera.fx"];
    params->cam_intrinsic.fy = cv_params["Camera.fy"];
    params->cam_intrinsic.cx = cv_params["Camera.cx"];
    params->cam_intrinsic.cy = cv_params["Camera.cy"];
    params->cam_intrinsic.s  = cv_params["Camera.s"];

    params->cam_intrinsic.k1 = cv_params["Camera.k1"];
    params->cam_intrinsic.k2 = cv_params["Camera.k2"];
    params->cam_intrinsic.p1 = cv_params["Camera.p1"];
    params->cam_intrinsic.p2 = cv_params["Camera.p2"];
    params->cam_intrinsic.k3 = cv_params["Camera.k3"];
    params->cam_intrinsic.width = cv_params["Camera.width"];
    params->cam_intrinsic.height = cv_params["Camera.height"];

    // Load wheel intrinsic.
    params->wheel_param.kl = cv_params["Wheel.kl"];
    params->wheel_param.kr = cv_params["Wheel.kr"];
    params->wheel_param.b = cv_params["Wheel.b"];
    params->wheel_param.noise_factor = cv_params["Wheel.noise_factor"];

    // Load extrinsic.
    cv::Mat cv_O_R_C;
    cv_params["Extrinsic.O_R_C"] >> cv_O_R_C;
    cv::Mat cv_O_p_C;
    cv_params["Extrinsic.O_p_C"] >> cv_O_p_C;
    cv::Mat cv_C_p_Gps;
    cv_params["Extrinsic.C_p_Gps"] >> cv_C_p_Gps;

    params->extrinsic.O_R_C << 
        cv_O_R_C.at<double>(0, 0), cv_O_R_C.at<double>(0, 1), cv_O_R_C.at<double>(0, 2), 
        cv_O_R_C.at<double>(1, 0), cv_O_R_C.at<double>(1, 1), cv_O_R_C.at<double>(1, 2), 
        cv_O_R_C.at<double>(2, 0), cv_O_R_C.at<double>(2, 1), cv_O_R_C.at<double>(2, 2), 
    params->extrinsic.O_p_C << cv_O_p_C.at<double>(0, 0), cv_O_p_C.at<double>(1, 0), cv_O_p_C.at<double>(2, 0);
    params->extrinsic.C_p_Gps << cv_C_p_Gps.at<double>(0, 0), cv_C_p_Gps.at<double>(1, 0), cv_C_p_Gps.at<double>(2, 0);

    // Visualization
    params->viz_config.cam_size = cv_params["viz_config.cam_size"];
    params->viz_config.cam_line_width = cv_params["viz_config.cam_line_width"];
    params->viz_config.point_size = cv_params["viz_config.point_size"];
    params->viz_config.wheel_frame_size = cv_params["viz_config.wheel_frame_size"];
    params->viz_config.view_point_x = cv_params["viz_config.view_point_x"];
    params->viz_config.view_point_y = cv_params["viz_config.view_point_y"];
    params->viz_config.view_point_z = cv_params["viz_config.view_point_z"];
    params->viz_config.view_point_f = cv_params["viz_config.view_point_f"];
    params->viz_config.img_height = cv_params["viz_config.img_height"];
    params->viz_config.img_width = cv_params["viz_config.img_width"];
    params->viz_config.max_traj_length = cv_params["viz_config.max_traj_length"];
    params->viz_config.max_num_features = cv_params["viz_config.max_num_features"];
    params->viz_config.max_gps_length = cv_params["viz_config.max_gps_length"];
    params->viz_config.gps_point_size = cv_params["viz_config.gps_point_size"];
    cv_params["viz_config.show_raw_odom"] >> params->viz_config.show_raw_odom;
    cv_params["viz_config.show_gps_points"] >> params->viz_config.show_gps_points;

    // Triangulator
    params->tri_config.max_proj_res = cv_params["tri_config.max_proj_res"];
    params->tri_config.min_dist = cv_params["tri_config.min_dist"];
    params->tri_config.max_dist = cv_params["tri_config.max_dist"];

    // Feature tracker
    params->tracker_config.max_num_corners = cv_params["tracker_config.max_num_corners"];
    params->tracker_config.quality_level = cv_params["tracker_config.quality_level"];
    params->tracker_config.min_dist = cv_params["tracker_config.min_dist"];

    // Visual Updater
    params->visual_updater_config.visual_noise = cv_params["visual_updater.visual_noise"];
    params->visual_updater_config.min_window_length = cv_params["visual_updater.min_window_length"];
    params->visual_updater_config.min_cam_dist_to_triangulate = cv_params["visual_updater.min_cam_dist_to_triangulate"];
    params->visual_updater_config.min_res_size = cv_params["visual_updater.min_res_size"];
    
    // Plane Updater
    params->plane_updater_config.plane_rot_noise = cv_params["plane_updater.plane_rot_noise"];
    params->plane_updater_config.plane_trans_noise = cv_params["plane_updater.plane_trans_noise"];

    // System config.
    params->sys_config.sliding_window_size = cv_params["sys_config.sliding_window_size"];
    cv_params["sys_config.compute_raw_odom"] >> params->sys_config.compute_raw_odom;
    cv_params["sys_config.enable_plane_update"] >> params->sys_config.enable_plane_update;
    cv_params["sys_config.enable_gps_update"] >> params->sys_config.enable_gps_update;
    
}

}  // namespace FilterFusion