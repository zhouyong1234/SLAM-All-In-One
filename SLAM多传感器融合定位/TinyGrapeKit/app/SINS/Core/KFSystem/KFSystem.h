#pragma once

#include <Base/SensorDataTypes.h>
#include <Base/InsState.h>
#include <Base/Index.h>
#include <Base/KFState.h>

namespace SINS {

class KFSystem {
public:
    KFSystem(double angle_random_walk, double vel_random_walk);

    bool ProcessImu(const ImuData::ConstPtr imu1, const ImuData::ConstPtr imu2, InsState *ins_state, KFState *kf_State);
    bool ProcessGnss(const GnssData::ConstPtr gnss_data, InsState *ins_state, KFState *kf_state);

private:
    Eigen::Matrix<double, 6, 6> imu_noise_;
};

}  // namespace