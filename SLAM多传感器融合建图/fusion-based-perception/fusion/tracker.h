// Copyright 2021 Sui Fang

#pragma once
#include <mutex>
#include "object.h"
#include "fusion_ekf.h"

namespace kit {
namespace perception {
namespace fusion {

class Tracker {
 public:
    Tracker()
        : global_obj_list_(
              std::make_shared<FusionObjectList>(FusionObjectList())),
          max_live_duration_(6),
          startup_duration_(6),
          dist_thres_(0.5),
          iou_thres_(0.5) {}

    bool Update(const FusionObjectListPtr &local_obj_list);

    bool Update(const LiDARObjectListPtr &lidar_obj_list,
            const std::map<size_t, int>& map);
    bool Update(const CameraObjectListPtr &camera_obj_list,
            const std::map<size_t, int>& map);
    bool Update(const RadarObjectListPtr &radar_obj_list,
            const std::map<size_t, int>& map);

    void GetFusionResults(const FusionObjectListPtr &res);
    void GetGlobalObjects(const FusionObjectListPtr &res);

 private:
    void PerodicallyUpdateLifeDuration();

    std::vector<std::shared_ptr<FusionEKF>> motion_filters_;

    std::mutex global_mutex_;
    int max_live_duration_ = 5;
    int startup_duration_ = 6;
    float dist_thres_ = 1.0;
    float iou_thres_ = 0.5;
    FusionObjectListPtr global_obj_list_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace kit
