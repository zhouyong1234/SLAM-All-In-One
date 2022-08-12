#pragma once

#include "Base/InsState.h"
#include "Base/Index.h"
#include "Base/KFState.h"

namespace SINS {

class Measurements {
public:
    static bool ComputePostionMeasurement(const InsState &ins_state, 
                               const Eigen::Matrix3d &Mpv,
                               const Eigen::Vector3d &lever_arm,
                               const Eigen::Vector3d &measure_pos, 
                               Eigen::Vector3d *zk,
                               Eigen::Matrix<double, 3, kKFStateDim> *H_wrt_kf_state,
                               Eigen::Matrix3d *H_wrt_lever_arm) ;

    static bool ComputeVelocityMeasurement(const InsState &ins_state,
                                const Eigen::Vector3d &lever_arm,
                                const Eigen::Vector3d &measure_vel,
                                Eigen::Vector3d *zk,
                                Eigen::Matrix<double, 3, kKFStateDim> *H_wrt_kf_state,
                                Eigen::Matrix3d *H_wrt_lever_arm);

};

} // namespace SINS