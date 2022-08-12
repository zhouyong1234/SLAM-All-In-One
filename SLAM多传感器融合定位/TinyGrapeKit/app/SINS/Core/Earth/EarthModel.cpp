#include "Earth/EarthModel.h"

#include <iostream>

namespace SINS {

EarthModel::EarthModel(const EarthParam &param) : param_(param) { 
    std::cout << std::fixed 
        << param.f << ", "
        << param.Re << ", "
        << param.wie << ", "
        << param.e << ", "
        << param.e2 << std::endl;
}

void EarthModel::GetRmRn(double latitude, double *Rm, double *Rn) {
    double sin_lat = std::sin(latitude);
    double sin_lat2 = sin_lat * sin_lat;
    double sq = 1. - param_.e2 * sin_lat2;
    *Rn = param_.Re / std::sqrt(sq);
    *Rm = *Rn * (1.0 - param_.e2) / sq;
}

Eigen::Vector3d EarthModel::GetGravity(double latitude, double height) {
    // TODO: Remove duplicate trigonometric operations
    double sin_lat = std::sin(latitude);
    double sin_lat2 = sin_lat * sin_lat;
    double sin_lat4 = sin_lat2 * sin_lat2;

    // Normal gravity: grs80
    double gn = param_.g0 * (1 + 5.27094e-3 * sin_lat2+ 2.32718e-5 * sin_lat4) - 3.086e-6 * height; 

    // TODO: Gravitational/Coriolis/Centripetal acceleration
    return Eigen::Vector3d(0.0, 0.0, -gn);
}

Eigen::Vector3d EarthModel::GetWnie(double latitude) {
    return Eigen::Vector3d(0.0, param_.wie * std::cos(latitude), param_.wie * std::sin(latitude));
}

double EarthModel::Getwie() {
    return param_.wie;
}

Eigen::Vector3d EarthModel::GetWnen(double east_vel, double north_vel, double latitude, double height, double Rm, double Rn) {
    return Eigen::Vector3d(
        -north_vel / (Rm + height),
        east_vel / (Rn + height),
        east_vel * std::tan(latitude) / (Rn + height)
    );
}

}  // namespace SINS