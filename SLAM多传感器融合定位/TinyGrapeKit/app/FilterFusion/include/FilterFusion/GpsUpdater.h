#pragma once

#include <deque>

#include <Eigen/Core>

#include <FilterFusion/State.h>
#include <TGK/BaseType/Measurement.h>

namespace FilterFusion {

class GpsUpdater {
public:
    GpsUpdater(const Eigen::Vector3d& C_p_Gps);

    bool UpdateState(const TGK::BaseType::GpsDataConstPtr gps_data, State* state);
    
    void SetInitLonLatHei(const Eigen::Vector3d& init_lon_lat_hei);

    bool GetInitLonLatHei(Eigen::Vector3d* init_lon_lat_hei);

private:
    Eigen::Vector3d C_p_Gps_;

    Eigen::Vector3d init_lon_lat_hei_;
    bool init_set = false;
    
    std::deque<TGK::BaseType::GpsDataConstPtr> gps_data_queue_;
};


Eigen::Vector3d ConvertLonLatHeiToENU(const Eigen::Vector3d& init_long_lat_hei, 
                                      const Eigen::Vector3d& point_long_lat_hei);

}  // namespace FilterFusion