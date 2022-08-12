#include "InsUpdate/InsUpdater.h" 
#include "Utils/SO3.h"

#include <iostream>

namespace SINS {

void InsUpdater::UpdateInsState(double delta_t,
                                const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2, 
                                const Eigen::Vector3d &delta_v1, const Eigen::Vector3d &delta_v2,
                                InsState *ins_state, bool fix_height) {
    // Compute earth params.
    if (ins_state->update_earth) {
        UpdateEarthParams(ins_state);
    } 
    
    // Cache last state.
    const Eigen::Vector3d last_vel = ins_state->velocity;
    const double last_hei = ins_state->lat_lon_hei[2];

    // Unbias.
    const Eigen::Vector3d &ub_delta_theta1 = delta_theta1 - ins_state->gyro_bias * delta_t * 0.5;
    const Eigen::Vector3d &ub_delta_theta2 = delta_theta2 - ins_state->gyro_bias * delta_t * 0.5;
    const Eigen::Vector3d &ub_delta_v1 = delta_v1 - ins_state->acc_bias * delta_t * 0.5;
    const Eigen::Vector3d &ub_delta_v2 = delta_v2 - ins_state->acc_bias * delta_t * 0.5;

    UpdateVelocity(delta_t, ub_delta_theta1, ub_delta_theta2, ub_delta_v1, ub_delta_v2, ins_state);
    UpdateOrientation(delta_t, ub_delta_theta1, ub_delta_theta2, ins_state);
    UpdatePosition(delta_t, last_vel, ins_state);

    // Fix height.
    if (fix_height) { ins_state->lat_lon_hei[2] = last_hei; }

    // Compute unbiased imu data.
    ins_state->ub_acc = 0.5 * delta_v2 / delta_t - ins_state->acc_bias;
    ins_state->ub_gyro = 0.5 * delta_theta2 / delta_t - ins_state->gyro_bias;
}

void InsUpdater::UpdateEarthParams(InsState *ins_state) {
    UpdateEarthParams(ins_state->lat_lon_hei[0], ins_state->lat_lon_hei[2], ins_state->velocity[0], ins_state->velocity[1], ins_state);
}

void InsUpdater::UpdateEarthParams(double latitude, double height, double east_vel, double north_vel,
                                   InsState *ins_state) {
    ins_state->gravity = EarthModel::Instance()->GetGravity(latitude, height);
    ins_state->Wnie = EarthModel::Instance()->GetWnie(latitude);
    EarthModel::Instance()->GetRmRn(latitude, &ins_state->Rm, &ins_state->Rn);
    ins_state->Wnen = EarthModel::GetWnen(east_vel, north_vel, latitude, height, ins_state->Rm, ins_state->Rn);
    ins_state->Wnin = ins_state->Wnie + ins_state->Wnen;
}

void InsUpdater::UpdateOrientation(double delta_t,
                                   const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2,
                                   InsState *ins_state) {
    // Rotation of the navigation frame.
    const Eigen::Vector3d phi_nn = -ins_state->Wnin * delta_t;
    Eigen::Quaterniond q_nn = QuatExp(phi_nn);

    // Rotation of the body.
    const Eigen::Vector3d phi_bb = delta_theta1 + delta_theta2 + (2.0 / 3.0) * delta_theta1.cross(delta_theta2);
    const Eigen::Quaterniond q_bb = QuatExp(phi_bb);

    // Put together.
    ins_state->orientation = q_nn * ins_state->orientation * q_bb;
    ins_state->orientation.normalize();
}

void InsUpdater::UpdateVelocity(double delta_t, 
                                const Eigen::Vector3d &delta_theta1, const Eigen::Vector3d &delta_theta2, 
                                const Eigen::Vector3d &delta_v1, const Eigen::Vector3d &delta_v2,
                                InsState *ins_state) {
    const Eigen::Vector3d delta_theta = delta_theta1 + delta_theta2;
    const Eigen::Vector3d delta_v = delta_v1 + delta_v2;
    const Eigen::Vector3d delta_v_rot = 0.5 * delta_theta.cross(delta_v);

    // Scul
    const Eigen::Vector3d delta_v_scul = (2.0 / 3.0) * (delta_theta1.cross(delta_v2) + delta_v1.cross(delta_theta2));
    // Vsf.
    const Eigen::Vector3d delta_v_sf = ins_state->orientation * (delta_v + delta_v_rot + delta_v_scul);
    // Corg
    const Eigen::Vector3d delta_v_corg = 
         (-(2.0 * ins_state->Wnie + ins_state->Wnen).cross(ins_state->velocity) + ins_state->gravity) * delta_t;
    
    // Put together
    ins_state->velocity = ins_state->velocity.eval() + delta_v_corg + delta_v_sf;
}

void InsUpdater::UpdatePosition(double delta_t, const Eigen::Vector3d &vel_m_1, InsState *ins_state) {
    Eigen::Matrix3d M;

    M << 0.0, 1.0 / (ins_state->Rm + ins_state->lat_lon_hei[2]), 0.0,
         1.0 / ((ins_state->Rn + ins_state->lat_lon_hei[2]) * std::cos(ins_state->lat_lon_hei[0])), 0.0, 0.0,
         0.0, 0.0, 1.0;

    ins_state->lat_lon_hei = ins_state->lat_lon_hei + M * (vel_m_1 + ins_state->velocity) * delta_t * 0.5;
}

}  // namespace SINS