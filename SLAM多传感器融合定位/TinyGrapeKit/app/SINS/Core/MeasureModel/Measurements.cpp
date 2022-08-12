
#include "MeasureModel/Measurements.h"
#include "Utils/Utils.h"

namespace SINS {

bool Measurements::ComputePostionMeasurement(const InsState &ins_state, 
                               const Eigen::Matrix3d &Mpv,
                               const Eigen::Vector3d &lever_arm,
                               const Eigen::Vector3d &measure_pos, 
                               Eigen::Vector3d *zk,
                               Eigen::Matrix<double, 3, kKFStateDim> *H_wrt_kf_state,
                               Eigen::Matrix3d *H_wrt_lever_arm) {
    const Eigen::Matrix3d Cnb = ins_state.orientation.toRotationMatrix();
    
    const Eigen::Vector3d exp_pos = ins_state.lat_lon_hei + Mpv * Cnb * lever_arm;
    *zk = exp_pos - measure_pos;

    // H wrt kf_state.
    H_wrt_kf_state->setZero();
    H_wrt_kf_state->block<3, 3>(0, kPosErrIdx) = Eigen::Matrix3d::Identity();

    // H wrt lever arm.
    *H_wrt_lever_arm = -Mpv * Cnb;

    return true;
}

bool Measurements::ComputeVelocityMeasurement(const InsState &ins_state,
                                const Eigen::Vector3d &lever_arm,
                                const Eigen::Vector3d &measure_vel,
                                Eigen::Vector3d *zk,
                                Eigen::Matrix<double, 3, kKFStateDim> *H_wrt_kf_state,
                                Eigen::Matrix3d *H_wrt_lever_arm) {
    const Eigen::Matrix3d Cnb = ins_state.orientation.toRotationMatrix();
    const Eigen::Vector3d exp_vel = ins_state.velocity + Cnb * SkewMat(ins_state.ub_gyro) * lever_arm;
    *zk = exp_vel - measure_vel;

    // H wrt kf state.
    H_wrt_kf_state->setZero();
    H_wrt_kf_state->block<3, 3>(0, kVelErrIdx) = Eigen::Matrix3d::Identity();

    // H wrt lever aram
    *H_wrt_lever_arm = -Cnb * SkewMat(ins_state.ub_gyro);

    return true;
}    

}  // namespace SINS

