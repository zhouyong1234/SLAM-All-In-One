#include <Simulation/VisualWheelCircleSim.h>

#include <Eigen/Dense>
#include <glog/logging.h>

#include <Const/Const.h>

namespace TGK {
namespace Simulation {

VisualWheelCircleSim::VisualWheelCircleSim(
    const std::shared_ptr<Camera::Camera> camera,
    const double kl, const double kr, const double b, 
    const Eigen::Matrix3d& O_R_C, 
    const Eigen::Vector3d& O_p_C,
    const double circle_radius, const int sample_num,
    const double view_range) 
    : camera_(camera), kl_(kl), kr_(kr), b_(b), 
      O_R_C_(O_R_C), O_p_C_(O_p_C),
      circle_radius_(circle_radius), sample_num_(sample_num), view_range_(view_range),
      frame_id_(0), 
      sample_dth_(2. * M_PI / static_cast<double>(sample_num)) { 
    CrateMapPoints();
}

bool VisualWheelCircleSim::SimOneFrame(double* timestamp, 
                                       Eigen::Matrix3d* G_R_O,
                                       Eigen::Vector3d* G_p_O,
                                       double* left_wheel, double* right_wheel, 
                                       std::vector<Eigen::Vector2d>* features, std::vector<long int>* feature_ids,
                                       cv::Mat* image) {
    if (frame_id_ >= sample_num_) { 
        LOG(WARNING) << "End of simulation";
        return false;
    }

    *timestamp = static_cast<double>(frame_id_);

    const double theta = frame_id_ * sample_dth_;
    ++frame_id_;

    // Ground truth.
    *G_p_O << circle_radius_ * std::cos(theta), circle_radius_ * std::sin(theta), 0.;
    constexpr double kHalfPi = 0.5 * M_PI;
    *G_R_O = Eigen::AngleAxisd(theta + kHalfPi, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Wheel measurement.
    *left_wheel = (circle_radius_ - 0.5 * b_) * theta / kl_;
    *right_wheel = (circle_radius_ + 0.5 * b_) * theta / kr_;

    // Image measurement.
    const Eigen::Matrix3d C_R_G = (*G_R_O * O_R_C_).transpose();
    const Eigen::Vector3d C_p_G = -C_R_G * (*G_p_O + *G_R_O * O_p_C_);

    // TODO: Use KD-Tree.
    feature_ids->clear();
    features->clear();
    for (long int i = 0; i < map_points_.size(); ++i) {
        const long int mpt_id = map_point_ids_[i];
        const Eigen::Vector3d G_p = map_points_[i];
        const Eigen::Vector3d C_p = C_R_G * G_p + C_p_G;

        if (C_p[2] > view_range_) { continue; }

        Eigen::Vector2d I_p;
        if (!camera_->CameraToImage(C_p, &I_p)) { continue; }

        features->push_back(I_p);
        feature_ids->push_back(mpt_id);
    }

    if (image != nullptr) {
        cv::Mat(camera_->height(), camera_->width(), CV_8UC3, cv::Scalar(0, 0, 0)).copyTo(*image);

        // Draw all features on image.
        for (size_t i = 0; i < features->size(); ++i) {
            const long int ft_id = (*feature_ids)[i];
            const Eigen::Vector2d pt = (*features)[i];
            const cv::Point2d cv_pt(pt[0], pt[1]);
            cv::circle(*image, cv_pt, 5, cv::Scalar(0, 255, 0), -1);
            cv::putText(*image, std::to_string(ft_id), cv_pt, 1, 1, cv::Scalar(0, 255, 0));
        } 
    }

    return true;
}

void VisualWheelCircleSim::Reset() { 
    frame_id_ = 0;
}

void VisualWheelCircleSim::GetAllMapPoint(std::vector<Eigen::Vector3d>* map_points, 
                                          std::vector<long int>* map_point_ids) {
    *map_points = map_points_;
    *map_point_ids = map_point_ids_;
}

void VisualWheelCircleSim::CrateMapPoints() {
    constexpr double dth = 2. * kDeg2Rad;
    constexpr double dalpha = 20. * kDeg2Rad;
    constexpr double map_r = 4.;

    long int mpt_id = 0;
    map_points_.clear();
    map_point_ids_.clear();
    // TODO: Reserve.
    
    for (double theta = 0.; theta < 2. * M_PI; theta += dth) {
        const Eigen::Matrix3d G_R_L = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Vector3d G_p_L(circle_radius_ * std::cos(theta), circle_radius_ * std::sin(theta), 0.);

        for (double alpha = 0.; alpha < 2. * M_PI; alpha += dalpha) {
            const Eigen::Vector3d L_p(map_r * std::cos(alpha), 0.,  map_r * std::sin(alpha));
            const Eigen::Vector3d G_p = G_R_L * L_p + G_p_L;
            map_points_.push_back(G_p);
            map_point_ids_.push_back(mpt_id);
            ++mpt_id;
        }
    }
}

}  // Simulation
}  // namespace TGK