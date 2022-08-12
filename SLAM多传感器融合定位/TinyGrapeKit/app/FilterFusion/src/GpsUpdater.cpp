#include <FilterFusion/GpsUpdater.h>

#include <Eigen/Dense>
#include <glog/logging.h>

#include <FilterFusion/UpdaterUtil.h>
#include <TGK/Util/Util.h>

#include <LocalCartesian.hpp>

namespace FilterFusion {

bool FindNearestTwoFrames(const double timestamp, const std::deque<CameraFramePtr> camera_frames,
                          CameraFramePtr* frame_a, CameraFramePtr* frame_b) {
    if (camera_frames.size() < 2 || 
        timestamp < camera_frames.front()->timestamp || 
        timestamp > camera_frames.back()->timestamp) {
        return false;
    }

    for (size_t i = 1; i < camera_frames.size(); ++i) {
        if (timestamp >= camera_frames[i-1]->timestamp && timestamp <= camera_frames[i]->timestamp) {
            *frame_a = camera_frames[i-1];
            *frame_b = camera_frames[i];
            return true;
        }
    }

    return false;
}

Eigen::Matrix3d LeftJacobian(const Eigen::Vector3d& w) {
    const double theta = w.norm();
    const double theta2 = theta * theta;
    const double theta3 = theta2 * theta;
    if (std::fabs(theta) < 1e-14 || std::fabs(theta2) < 1e-14|| std::fabs(theta3)< 1e-14) {
        return Eigen::Matrix3d::Identity();
    }

    const Eigen::Matrix3d w_hat = TGK::Util::Skew(w);
    return Eigen::Matrix3d::Identity() +  w_hat * (1. - std::cos(theta)) / theta2 + 
            w_hat * w_hat * (theta - std::sin(theta)) / theta3;
}

Eigen::Matrix3d RightJacobian(const Eigen::Vector3d& w) {
    return LeftJacobian(-w);
}

bool ComputeGpsConstraintResidualJacobian(const Eigen::Vector3d& C_p_Gps,
                                          const double ta, const Eigen::Matrix3d& G_R_Ca, Eigen::Vector3d& G_p_Ca, 
                                          const double tb, const Eigen::Matrix3d& G_R_Cb, Eigen::Vector3d& G_p_Cb, 
                                          const double tg, const Eigen::Vector3d& G_p_Gps,
                                          Eigen::Vector3d* res, 
                                          Eigen::Matrix<double, 3, 6>* J_wrt_Ta,
                                          Eigen::Matrix<double, 3, 6>* J_wrt_Tb) {
    // Compute residual.
    if (tb - ta < 1e-6) {
        LOG(ERROR) << "[ComputeGpsConstraintResidualJacobian]: Time inverval between two camera frame is too small.";
        return false;
    }

    // 1. Compute residual.
    const double lambda = (tg - ta) / (tb - ta);
    const Eigen::AngleAxisd a_R_b_ax = Eigen::AngleAxisd(G_R_Ca.transpose() * G_R_Cb);
    const double a_angle_b = a_R_b_ax.angle();
    const Eigen::Vector3d a_axis_b = a_R_b_ax.axis();
    const Eigen::Vector3d tau = a_R_b_ax.angle() * a_R_b_ax.axis(); // Delta angle axis.

    const Eigen::Matrix3d a_R_g = Eigen::AngleAxisd(lambda * a_angle_b, a_axis_b).toRotationMatrix();
    const Eigen::Matrix3d G_R_Cg = G_R_Ca * a_R_g;
    const Eigen::Vector3d G_p_Cg = lambda * G_p_Cb + (1 - lambda) * G_p_Ca;

    // 计算残差
    const Eigen::Vector3d exp_G_p_Gps = G_p_Cg + G_R_Cg * C_p_Gps;
    *res = G_p_Gps - exp_G_p_Gps;

    // 2. Compute Jacobian.
    Eigen::Matrix<double, 3, 6> J_wrt_Tg;
    // 残差对tg时刻相机位姿的雅克比
    J_wrt_Tg << -G_R_Cg * TGK::Util::Skew(C_p_Gps), Eigen::Matrix3d::Identity();

    const Eigen::Matrix3d J_l_lambda_tau = LeftJacobian(lambda * tau);
    const Eigen::Matrix3d J_l_tau_inv = LeftJacobian(tau).inverse();
    const Eigen::Matrix3d J_r_lambda_tau = RightJacobian(lambda * tau);
    const Eigen::Matrix3d J_r_tau_inv = RightJacobian(tau).inverse();
    Eigen::Matrix<double, 6, 6> J_Tg_wrt_Ta;
    J_Tg_wrt_Ta.setZero();
    J_Tg_wrt_Ta.block<3, 3>(0, 0) = a_R_g.transpose() * (Eigen::Matrix3d::Identity() - lambda * J_l_lambda_tau * J_l_tau_inv);
    J_Tg_wrt_Ta.block<3, 3>(3, 3) = (1- lambda) * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> J_Tg_wrt_Tb;
    J_Tg_wrt_Tb.setZero();
    J_Tg_wrt_Tb.block<3, 3>(0, 0) = lambda * J_r_lambda_tau * J_r_tau_inv;
    J_Tg_wrt_Tb.block<3, 3>(3, 3) = lambda * Eigen::Matrix3d::Identity();

    *J_wrt_Ta = J_wrt_Tg * J_Tg_wrt_Ta;
    *J_wrt_Tb = J_wrt_Tg * J_Tg_wrt_Tb;

    return true;                              
}

Eigen::Vector3d ConvertLonLatHeiToENU(const Eigen::Vector3d& init_long_lat_hei, 
                                      const Eigen::Vector3d& point_long_lat_hei) {
    Eigen::Vector3d point_enu;
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_long_lat_hei(1), init_long_lat_hei(0), init_long_lat_hei(2));
    local_cartesian.Forward(point_long_lat_hei(1), point_long_lat_hei(0), point_long_lat_hei(2), 
                            point_enu.data()[0], point_enu.data()[1], point_enu.data()[2]);
    
    return point_enu;
}

Eigen::Vector3d ConvertENUToLonLatHei(const Eigen::Vector3d& init_long_lat_hei, 
                                      const Eigen::Vector3d& point_enu) {
    Eigen::Vector3d point_long_lat_hei;
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_long_lat_hei(1), init_long_lat_hei(0), init_long_lat_hei(2));
    local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2), 
                            point_long_lat_hei.data()[1], point_long_lat_hei.data()[0], point_long_lat_hei.data()[2]);                            
    
    return point_long_lat_hei;
}


GpsUpdater::GpsUpdater(const Eigen::Vector3d& C_p_Gps) : C_p_Gps_(C_p_Gps) { }

bool GpsUpdater::UpdateState(const TGK::BaseType::GpsDataConstPtr gps_data, State* state) {
    if (state->camera_frames.size() < 2) {
        LOG(WARNING) << "[GpsUpdater::UpdateState]: No enough camera clones in state.";
        return false;
    }

    if (!init_set) {
        LOG(WARNING) << "[GpsUpdater::UpdateState]: Init lon lat not set yet.";
        return false;
    }

    const double min_timestamp = state->camera_frames.front()->timestamp;
    const double max_timestamp = state->camera_frames.back()->timestamp;
    if (gps_data->timestamp < min_timestamp) {
        LOG(WARNING) << "[GpsUpdater::UpdateState]: Gps timestamp is too small. Return.";
        return true;
    }

    // Push to buffer.
    gps_data_queue_.push_back(gps_data);

    while(!gps_data_queue_.empty()) {
        const auto& cur_gps_data = gps_data_queue_.front();
        if (cur_gps_data->timestamp < min_timestamp) {
            gps_data_queue_.pop_front();
            continue;
        }

        if (cur_gps_data->timestamp > max_timestamp) {
            break;
        }

        gps_data_queue_.pop_front();
        
        // Find G_T_Ca, G_T_Cb.
        CameraFramePtr frame_a, frame_b;
        if (!FindNearestTwoFrames(cur_gps_data->timestamp, state->camera_frames, &frame_a, &frame_b)) {
            LOG(ERROR) << "[GpsUpdater::UpdateState]: Failed to FindNearestTwoFrames.";
            continue;
        }

        // WGS84 TO ENU.
        const Eigen::Vector3d G_p_Gps = ConvertLonLatHeiToENU(init_lon_lat_hei_, cur_gps_data->lon_lat_hei);

        /// Compute Jacobian.
        Eigen::Vector3d res;
        Eigen::Matrix<double, 3, 6> J_wrt_Ta, J_wrt_Tb;
        if (!ComputeGpsConstraintResidualJacobian(C_p_Gps_, 
            frame_a->timestamp, frame_a->G_R_C, frame_a->G_p_C,
            frame_b->timestamp, frame_b->G_R_C, frame_b->G_p_C,
            cur_gps_data->timestamp, G_p_Gps, &res, &J_wrt_Ta, &J_wrt_Tb)) {
            LOG(WARNING) << "[GpsUpdater::UpdateState]: Failed to ComputeGpsConstraintResidualJacobian";
            continue;
        }
        const size_t state_size = state->covariance.rows();
        Eigen::MatrixXd Hx(3, state_size);
        Hx.setZero();
        Hx.block<3, 6>(0, frame_a->state_idx) = J_wrt_Ta;
        Hx.block<3, 6>(0, frame_b->state_idx) = J_wrt_Tb;

        // TODO: Adjust noise.
        Eigen::Matrix3d cov = cur_gps_data->cov;
        cov *= 0.1;
        cov(2, 2) = 1e6; // Do not use height.

        /// EKF update.
        EKFUpdate(Hx, res, cov, state);
    }

    return true;
}
    
void GpsUpdater::SetInitLonLatHei(const Eigen::Vector3d& init_lon_lat_hei) {
    init_lon_lat_hei_ = init_lon_lat_hei;
    init_set = true;
}

bool GpsUpdater::GetInitLonLatHei(Eigen::Vector3d* init_lon_lat_hei) {
    *init_lon_lat_hei = init_lon_lat_hei_;
    return init_set;
}

}  // namespace FilterFusion