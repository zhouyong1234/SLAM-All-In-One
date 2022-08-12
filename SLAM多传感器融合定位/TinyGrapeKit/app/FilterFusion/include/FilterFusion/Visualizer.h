#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace FilterFusion {

class Visualizer {
public:
    struct Config {
        double cam_size = 1.5;
        double cam_line_width = 3.;
        double point_size = 2.;
        double wheel_frame_size = 2.;
        double view_point_x = 0.;
        double view_point_y = 0.;
        double view_point_z = 200.;
        double view_point_f = 500.;

        double img_height = 140;
        double img_width = 320;

        int max_traj_length = 100000;
        int max_num_features = 5000;
        
        int max_gps_length = 10000;
        double gps_point_size = 5.;

        bool show_raw_odom = true;
        bool show_gps_points = true;
    };

    Visualizer(const Config& config);
    ~Visualizer() {
        running_flag_ = false;
        viz_thread_->join();
    }

    void DrawCameras(const std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>& camera_poses);

    void DrawWheelPose(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O);

    void DrawFeatures(const std::vector<Eigen::Vector3d>& features);

    void DrawColorImage(const cv::Mat& image);
    void DrawImage(const cv::Mat& image,
                   const std::vector<Eigen::Vector2d>& tracked_fts, 
                   const std::vector<Eigen::Vector2d>& new_fts);

    void DrawGroundTruth(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O);

    void DrawWheelOdom(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O);

    void DrawGps(const Eigen::Vector3d& G_p_Gps);

private:
    void Run();

    void DrawOneCamera(const Eigen::Matrix3d& G_R_C, const Eigen::Vector3d& G_p_C);
    void DrawCameras();
    void DrawTraj(const std::deque<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>& traj_data);
    void DrawFeatures();
    void DrawWheeFrame(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O);
    void DrawWheeFrame();
    void DrawGpsPoints();

    const Config config_;
    
    // Thread.
    std::shared_ptr<std::thread> viz_thread_;
    std::atomic<bool> running_flag_;

    // Data buffer.
    std::mutex data_buffer_mutex_;
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> camera_poses_;
    std::deque<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> wheel_traj_;
    std::deque<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> gt_wheel_traj_;
    std::deque<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> wheel_odom_traj_;
    std::deque<Eigen::Vector3d> features_;
    std::deque<Eigen::Vector3d> gps_points_;

    cv::Mat image_;
};

}  // namespace FilterFusion