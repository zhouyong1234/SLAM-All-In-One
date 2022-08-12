#include "ErrorModel/ErrorModel.h"
#include "Utils/Utils.h"
#include "Earth/EarthModel.h"

namespace SINS {

Eigen::Matrix3d ComputeMaa(const Eigen::Vector3d &wnin) {
   return -SkewMat(wnin);
}

Eigen::Matrix3d ComputeMav(double Rmh, double Rnh, double latitude) {
    Eigen::Matrix3d mat;

    mat << 0., 1.0 / Rmh, 0.0,
        1. / Rnh, 0.0, 0.0,
        std::tan(latitude) / Rnh, 0.0, 0.0; 

    return mat;
}

Eigen::Matrix3d ComputeM1(double wie, double latitude) {
    Eigen::Matrix3d mat;

    mat << 0.0, 0.0, 0.0, 
           -wie * std::sin(latitude), 0.0, 0.0,
           wie * std::cos(latitude), 0.0, 0.0;

    return mat;
}

Eigen::Matrix3d ComputeM2(double east_vel, double north_vel, double Rmh, double Rnh, double latitude) {
    Eigen::Matrix3d mat;

    mat << 0.0, 0.0, north_vel / (Rmh * Rmh),
           0.0, 0.0, -east_vel / (Rnh * Rnh),
           east_vel / (std::cos(latitude) * std::cos(latitude) * Rnh), 0.0, -east_vel * std::tan(latitude) / (Rnh * Rnh);
    
    return mat;
}   

Eigen::Matrix3d ComputeM3() {
    return Eigen::Matrix3d::Zero();
}

Eigen::Matrix3d ComputeMap(const Eigen::Matrix3d &M1, const Eigen::Matrix3d &M2) {
    return M1 + M2;
}

Eigen::Matrix3d ComputeMab(const Eigen::Matrix3d &Cnb) {
    return -Cnb;
}

Eigen::Matrix3d ComputeMva(const Eigen::Quaterniond &qnb, const Eigen::Vector3d &fbsf) {
    return SkewMat(qnb.conjugate() * fbsf);
}

Eigen::Matrix3d ComputeMvv(const Eigen::Matrix3d &Mav, const Eigen::Vector3d &velocity, const Eigen::Vector3d &Wnie, const Eigen::Vector3d &Wnen) {
    return SkewMat(velocity) * Mav - SkewMat(2.0 * Wnie + Wnen);
}

Eigen::Matrix3d ComputeMvp(const Eigen::Vector3d &velocity, const Eigen::Matrix3d &M1, const Eigen::Matrix3d &M2, const Eigen::Matrix3d &M3) {
    return SkewMat(velocity) * (2.0 * M1 + M2) + M3;
}

Eigen::Matrix3d ComputeMvab(const Eigen::Matrix3d &Cnb) {
    return Cnb;
}

Eigen::Matrix3d ComputeMpv(double Rmh, double Rnh, double latitude) {
    Eigen::Matrix3d mat;

    mat << 0.0, 1. / Rmh, 0.0,
           sec(latitude) / Rnh, 0.0, 0.0,
           0.0, 0.0, 1.0;
    
    return mat;
}

Eigen::Matrix3d ComputeMpp(double east_vel, double north_vel, double Rmh, double Rnh, double latitude) {
    Eigen::Matrix3d mat;

    mat << 0.0, 0.0, -north_vel / (Rmh * Rmh),
           east_vel * sec(latitude) * std::tan(latitude) / Rnh, 0.0, -east_vel * sec(latitude) / (Rnh * Rnh),
           0.0, 0.0, 0.0;

    return mat;
}

KFMat ErrorModel::ComputeFMatrix(const InsState &ins_state) {
    KFMat F;
    F.setZero();

    double Rmh = ins_state.Rm + ins_state.lat_lon_hei[2];
    double Rnh = ins_state.Rn + ins_state.lat_lon_hei[2];
    double latitude = ins_state.lat_lon_hei[0];
    double east_vel = ins_state.velocity.x();
    double north_vel = ins_state.velocity.y();
    double wie = EarthModel::Instance()->Getwie();
    const Eigen::Matrix3d Cnb = ins_state.orientation.toRotationMatrix();
    const Eigen::Vector3d fbsf = ins_state.ub_acc;

    const Eigen::Matrix3d M1 = ComputeM1(wie, latitude);
    const Eigen::Matrix3d M2 = ComputeM2(east_vel, north_vel, Rmh, Rnh, latitude);
    const Eigen::Matrix3d M3 = ComputeM3();

    F.block<3, 3>(kAttErrIdx, kAttErrIdx) = ComputeMaa(ins_state.Wnin);
    const Eigen::Matrix3d Mav = ComputeMav(Rmh, Rnh, latitude);
    F.block<3, 3>(kAttErrIdx, kVelErrIdx) = Mav;
    F.block<3, 3>(kAttErrIdx, kPosErrIdx) = ComputeMap(M1, M2);
    F.block<3, 3>(kAttErrIdx, kGyroBiasErrIdx) = ComputeMab(Cnb);

    F.block<3, 3>(kVelErrIdx, kAttErrIdx) = ComputeMva(ins_state.orientation, fbsf);
    F.block<3, 3>(kVelErrIdx, kVelErrIdx) = ComputeMvv(Mav, ins_state.velocity, ins_state.Wnie, ins_state.Wnen);
    F.block<3, 3>(kVelErrIdx, kPosErrIdx) = ComputeMvp(ins_state.velocity, M1, M2, M3);
    F.block<3, 3>(kVelErrIdx, kAccBiasErrIdx) = ComputeMvab(Cnb);

    F.block<3, 3>(kPosErrIdx, kVelErrIdx) = ComputeMpv(Rmh, Rnh, latitude);
    F.block<3, 3>(kPosErrIdx, kPosErrIdx) = ComputeMpp(east_vel, north_vel, Rmh, Rnh, latitude);

    return F;
}

KFMat ErrorModel::ComputePhiMatrix(const InsState &ins_state) {
    return KFMat::Identity() + ComputeFMatrix(ins_state);
}

Eigen::Matrix<double, kKFStateDim, 6> ErrorModel::ComputeGMatrix(const InsState &ins_state) {
    Eigen::Matrix<double, kKFStateDim, 6> G;
    G.setZero();

    const Eigen::Matrix3d Cnb = ins_state.orientation.toRotationMatrix();
    G.block<3, 3>(kAttErrIdx, kGyroNoiseIdx) = -Cnb;
    G.block<3, 3>(kVelErrIdx, kAccNoiseIdx) = Cnb;

    return G;
}



}  // namespace SINS