// Copyright 2021 Sui Fang

#include "fusion_wrapper.h"

namespace kit {
namespace perception {
namespace fusion {

FusionWrapper::FusionWrapper(const std::string& config_file) :
    predictor_(std::shared_ptr<fusion::Predictor>(new fusion::Predictor())),
    matcher_(std::shared_ptr<fusion::Matcher>(new fusion::Matcher(config_file))),
    tracker_(std::shared_ptr<fusion::Tracker>(new fusion::Tracker())) {
}

bool FusionWrapper::Update(const FramePtr &frame) {

    fusion::FusionObjectListPtr global_obj_list =
        std::make_shared<fusion::FusionObjectList>(fusion::FusionObjectList());


    // fuse lidar measurement
    if (frame->lidar_objs->objs.size() > 0) {
        std::cout << "updating with lidar objects..." << std::endl;
        tracker_->GetGlobalObjects(global_obj_list);
        if (!predictor_->Predict(global_obj_list, frame->lidar_objs->time_ns)) {
            std::cout << "predict error for lidar measurement." << std::endl;
            return false; 
        }
        // map of lidar obj idx and global obj idx
        std::map<size_t, int> local_global_map;
        if (!matcher_->Match(frame->lidar_objs, global_obj_list, local_global_map)) {
            std::cout << "match failed for lidar and global." << std::endl;
            return false;
        }
        // update global object with lidar measurements
        tracker_->Update(frame->lidar_objs, local_global_map);
    }

    // TODO: radar will be available soon
    // if (frame->radar_objs->size() > 0) {
    //     PreProcess(frame->radar_objs, extrinsic_radar_to_baselink_);
    //     tracker_->GetGlobalObjects(global_obj_list);
    //     if (!predictor->Predict(global_obj_list, frame->lidar_objs->time_ns)) {
    //         std::cout << "predict error for radar measurement." << std::endl;
    //         return false; 
    //     }
    //     std::map<size_t, int> associate_arr;
    //     matcher_->Match(frame->radar_objs, global_obj_list, associate_arr);
    //     tracker_->Update(frame->radar_objs, global_obj_list, associate_arr);
    // }

    if (frame->camera_objs->objs.size() > 0) {
        std::cout << "updating with camera objects..." << std::endl;
        tracker_->GetGlobalObjects(global_obj_list);
        if (!predictor_->Predict(global_obj_list, frame->camera_objs->time_ns)) {
            std::cout << "predict error for camera measurement." << std::endl;
            return false; 
        }
        // map of camera obj idx and global obj idx
        std::map<size_t, int> local_global_map;
        if (!matcher_->Match(frame->camera_objs, global_obj_list, local_global_map)) {
            std::cout << "match failed for camera and global." << std::endl;
            return false;
        }
        tracker_->Update(frame->camera_objs, local_global_map);
    }

    return true;
}

bool FusionWrapper::GetFusionResult(const FusionObjectListPtr &res) {
    tracker_->GetFusionResults(res);
    return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace kit
