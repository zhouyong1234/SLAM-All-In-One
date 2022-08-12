#include <FilterFusion/PlaneUpdater.h>

#include <TGK/Util/Util.h>
#include <FilterFusion/UpdaterUtil.h>

namespace FilterFusion {
namespace {
    void ComputePlaneConstraintResidualJacobian(const Eigen::Matrix3d& G_R_O, const Eigen::Vector3d& G_p_O,
                                                Eigen::Vector3d* res, Eigen::Matrix<double, 3, 6>* H) {
        const Eigen::Matrix3d pi_R_G = Eigen::Matrix3d::Identity();
        const double pi_z_G = 0.;
        
        Eigen::Matrix<double, 2, 3> Lambda;
        Lambda << 1., 0., 0.,
                0., 1., 0.;
        const Eigen::Vector3d e3(0., 0., 1.);

        res->head<2>() = -Lambda * pi_R_G * G_R_O * e3;
        (*res)[2] = pi_z_G + e3.transpose() * pi_R_G * G_p_O;

        H->setZero();
        H->block<2, 3>(0, 0) = -Lambda * pi_R_G * G_R_O * TGK::Util::Skew(e3);
        H->block<1, 3>(2, 3) = -e3.transpose() * pi_R_G;
    }
}  // namespace 

PlaneUpdater::PlaneUpdater(const Config& config) : config_(config) { }

bool PlaneUpdater::UpdateState(State* state) {
    // Plane constraint update.
    Eigen::Vector3d plane_res;
    Eigen::Matrix<double, 3, 6> plane_H;
    ComputePlaneConstraintResidualJacobian(state->wheel_pose.G_R_O, state->wheel_pose.G_p_O, &plane_res, &plane_H);
    Eigen::MatrixXd big_plane_H(3, state->covariance.rows());
    big_plane_H.setZero();
    big_plane_H.block<3, 6>(0, 0) = plane_H;

    Eigen::Matrix3d plane_noise = Eigen::Matrix3d::Identity();
    plane_noise(0, 0) = config_.plane_rot_noise;
    plane_noise(1, 1) = config_.plane_rot_noise;
    plane_noise(2, 2) = config_.plane_trans_noise;
    EKFUpdate(big_plane_H, plane_res, plane_noise, state);

    return true;
}

}  // namespace FilterFusion