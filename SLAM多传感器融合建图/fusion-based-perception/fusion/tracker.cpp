// Copyright 2021 Sui Fang

#include <iostream>

#include "tracker.h"
#include "utils.h"
#include "state.h"

namespace kit {
namespace perception {
namespace fusion {

bool Tracker::Update(const LiDARObjectListPtr &lidar_obj_list,
        const std::map<size_t, int>& map) {
    for (auto m : map) {
        if (m.second >= 0) {
            Measurement lidar_mea;
            const auto &lidar_obj = lidar_obj_list->objs[m.first];
            const auto &global_obj = global_obj_list_->objs[m.second];
            lidar_mea.sensor_type = SensorType::LIDAR;
            Eigen::VectorXd meas(5);
            meas << lidar_obj->x,
                lidar_obj->y,
                lidar_obj->velo_x,
                lidar_obj->velo_y,
                lidar_obj->label;
            // std::cout << "lidar measurement: " << lidar_obj->ToString() << std::endl;
            // std::cout << "global object: " << global_obj->ToString() << std::endl;
            lidar_mea.time_ns = lidar_obj->time_ns;
            lidar_mea.meas = meas;
            State state;
            meas << global_obj->x,
                global_obj->y,
                global_obj->velo_x,
                global_obj->velo_y,
                global_obj->label;
            state.time_ns = global_obj->time_ns;
            state.meas = meas;
            // update fusion_obj with lidar_measurement using KalmanFilter
            motion_filters_[m.second]->UpdateMotion(state, lidar_mea);
            global_obj->x = motion_filters_[m.second]->GetState()[0];
            global_obj->y = motion_filters_[m.second]->GetState()[1];
            global_obj->z = lidar_obj->z;
            global_obj->velo_x = motion_filters_[m.second]->GetState()[2];
            global_obj->velo_y = motion_filters_[m.second]->GetState()[3];
            global_obj->label = motion_filters_[m.second]->GetState()[4];
            global_obj->length = lidar_obj->length;
            global_obj->width = lidar_obj->width;
            global_obj->height = lidar_obj->height;
            int dura = global_obj_list_->objs[m.second]->life_duration;
            global_obj_list_->objs[m.second]->life_duration =  (dura < startup_duration_ ? dura+1 : startup_duration_);
            std::cout << "updating exist global object: " << global_obj_list_->objs[m.second]->ToString() << std::endl;
        } else {
            // create a new global object with lidar object
            FusionObjectPtr new_obj =
                std::make_shared<FusionObject>(FusionObject());
            const auto &lidar_obj = lidar_obj_list->objs[m.first];
            new_obj->time_ns = lidar_obj->time_ns;
            // TODO: we should maintain a global unique id for new object
            new_obj->id = lidar_obj->id;
            new_obj->x = lidar_obj->x;
            new_obj->y = lidar_obj->y; new_obj->z = lidar_obj->z;
            new_obj->length = lidar_obj->length;
            new_obj->width = lidar_obj->width;
            new_obj->height = lidar_obj->height;
            new_obj->velo_x = lidar_obj->velo_x;
            new_obj->velo_y = lidar_obj->velo_y;
            new_obj->velo_z = lidar_obj->velo_z;
            new_obj->label = lidar_obj->label;
            new_obj->confidence = lidar_obj->confidence;
            new_obj->life_duration = startup_duration_;
            global_obj_list_->objs.push_back(new_obj);
            std::shared_ptr<FusionEKF> motion_filter =
                std::make_shared<FusionEKF>(FusionEKF());
            motion_filters_.push_back(motion_filter);
            std::cout << "create new global object: " << new_obj->ToString() << std::endl;
        }
        global_obj_list_->time_ns = lidar_obj_list->time_ns;
    }

    return true;
}

bool Tracker::Update(const RadarObjectListPtr &radar_obj_list,
        const std::map<size_t, int>& map) {
    // TODO: radar will be available soon
    for (auto m : map) {
        if (m.second >= 0) {
            Measurement radar_mea;
            const auto& radar_obj = radar_obj_list->objs[m.first];
            const auto& global_obj = global_obj_list_->objs[m.second];
            radar_mea.sensor_type = SensorType::RADAR;
            Eigen::VectorXd meas(5);
            meas << radar_obj->x,
                 radar_obj->y,
                 radar_obj->velo_x,
                 radar_obj->velo_y,
                 0;
            radar_mea.meas = meas;

            State state;
            state.time_ns = global_obj->time_ns;
            state.meas = meas;
            // Job3: update fusion_obj with lidar_measurement using KalmanFilter
            motion_filters_[m.second]->UpdateMotion(state, radar_mea);
            global_obj->x = motion_filters_[m.second]->GetState()[0];
            global_obj->y = motion_filters_[m.second]->GetState()[1];
            global_obj->velo_x = motion_filters_[m.second]->GetState()[2];
            global_obj->velo_y = motion_filters_[m.second]->GetState()[3];
            global_obj->label = motion_filters_[m.second]->GetState()[4];
        } 
    }

    return true;
}

bool Tracker::Update(const CameraObjectListPtr &camera_obj_list,
        const std::map<size_t, int>& map) {
    for (auto m : map) {
        if (m.second >= 0) {
            const auto& cam_obj = camera_obj_list->objs[m.first];
            Measurement cam_mea;
            cam_mea.time_ns = cam_obj->time_ns;
            cam_mea.sensor_type = SensorType::CAMERA;
            Eigen::VectorXd meas(5);
            meas << cam_obj->ux, cam_obj->vy, cam_obj->width, cam_obj->height, float(cam_obj->label);
            cam_mea.meas = meas;
            std::cout << "cam mea: " << cam_obj->ToString() << std::endl;

            auto& global_obj = global_obj_list_->objs[m.second];
            State state;
            state.time_ns = global_obj->time_ns;
            meas << global_obj->ux, global_obj->vy, global_obj->width_2d, global_obj->height_2d, float(global_obj->label); 
            state.meas = meas;
            std::cout << "global obj: " << global_obj->ToString() << std::endl;

            // Job4: update fusion_obj with camera_measurement using KalmanFilter
            motion_filters_[m.second]->UpdateAttr(state, cam_mea);
            global_obj->ux = motion_filters_[m.second]->GetAttrState()[0];
            global_obj->vy = motion_filters_[m.second]->GetAttrState()[1];
            global_obj->width_2d = motion_filters_[m.second]->GetAttrState()[2];
            global_obj->height_2d = motion_filters_[m.second]->GetAttrState()[3];
            global_obj->label = motion_filters_[m.second]->GetAttrState()[4];
            std::cout << "after updated: " << cam_obj->ToString() << std::endl;
        } 
    }

    return true;
}

bool Tracker::Update(const FusionObjectListPtr& local_obj_list) {
    {
        std::lock_guard<std::mutex> guard(global_mutex_);
        for (size_t i = 0; i < local_obj_list->objs.size(); ++i) {
            const auto& local = local_obj_list->objs[i];
            bool local_found = false;
            for (size_t j = 0; j < global_obj_list_->objs.size(); ++j) {
                auto& global = global_obj_list_->objs[j];
                float dist = DistanceIn3D(local, global);
                float iou = IoUIn2D(local, global);
                if (dist < dist_thres_ && iou > iou_thres_) {
                    global->time_ns = local->time_ns;
                    global->id = local->id;
                    global->x = local->x;
                    global->y = local->y;
                    global->velo_x = local->velo_x;
                    global->velo_y = local->velo_y;
                    global->label = local->label;
                    global->ux = local->ux;
                    global->vy = local->vy;
                    global->width_2d = local->width_2d;
                    global->height_2d = local->height_2d;
                    // *global = *local;
                    if (global->life_duration < max_live_duration_) {
                        global->life_duration = startup_duration_;
                    }
                    local_found = true;
                } else if (iou > iou_thres_ && dist <= dist_thres_) {
                    continue;
                }
            }
            if (!local_found) {
                FusionObjectPtr new_obj =
                    std::make_shared<FusionObject>(FusionObject());
                *new_obj = *local;
                new_obj->life_duration = startup_duration_;
                global_obj_list_->objs.push_back(new_obj);
            }
        }
    }
    return true;
}

void Tracker::PerodicallyUpdateLifeDuration() {
    for (size_t i = 0; i < global_obj_list_->objs.size(); ++i) {
        auto& global_obj = global_obj_list_->objs[i];
        --global_obj->life_duration;
        if (global_obj->life_duration <= 0) {
            global_obj_list_->objs.erase(global_obj_list_->objs.begin() + i);
        }
    }
}

void Tracker::GetGlobalObjects(const FusionObjectListPtr& res) {
    res->objs.clear();
    {
        std::lock_guard<std::mutex> guard(global_mutex_);
        res->time_ns = global_obj_list_->time_ns;
        for (size_t i = 0; i < global_obj_list_->objs.size(); ++i) {
            FusionObjectPtr new_obj =
                std::make_shared<FusionObject>(FusionObject());
            *new_obj = *global_obj_list_->objs[i];
            res->objs.push_back(new_obj);
        }
    }
}
void Tracker::GetFusionResults(const FusionObjectListPtr& res) {
    res->objs.clear();
    {
        std::lock_guard<std::mutex> guard(global_mutex_);
        PerodicallyUpdateLifeDuration();
        res->time_ns = global_obj_list_->time_ns;
        for (size_t i = 0; i < global_obj_list_->objs.size(); ++i) {
            FusionObjectPtr new_obj =
                std::make_shared<FusionObject>(FusionObject());
            *new_obj = *global_obj_list_->objs[i];
            res->objs.push_back(new_obj);
        }
    }
}

}  // namespace fusion
}  // namespace perception
}  // namespace kit
