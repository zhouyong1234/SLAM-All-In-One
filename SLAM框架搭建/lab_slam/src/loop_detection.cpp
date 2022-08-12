//
// Created by jin on 2021/12/23.
//
#include "loop_detection.h"

LoopDetection::LoopDetection() : positions_(new pcl::PointCloud<pcl::PointXYZ>), near_frame_num_(8){

}
bool LoopDetection::alignKeyFrames(const std::vector<KeyFramePtr> &key_frames,
                                              LoopClosurePair &loop_pairs) {
    PointCloudXYZI tmp_cloud;
    Eigen::Affine3d tmp_delta;

    PointCloudXYZIPtr target_cloud(new PointCloudXYZI);
    int target_index = loop_pairs.target_index;
    Eigen::Affine3d target_pose = key_frames.at(target_index)->pose_;
    for(int index = target_index - near_frame_num_; index < target_index + near_frame_num_; ++index){
        if(index < 0 || index >= key_frames.size()) continue;
//        *target_cloud += *(key_frames.at(index)->corner_cloud_);
//        *target_cloud += *(key_frames.at(index)->plane_cloud_);
        tmp_cloud.clear();
        tmp_cloud += *(key_frames.at(index)->corner_cloud_);
        tmp_cloud += *(key_frames.at(index)->plane_cloud_);
        tmp_delta = target_pose.inverse() * key_frames.at(index)->pose_;
        pcl::transformPointCloud(tmp_cloud, tmp_cloud, tmp_delta);
        *target_cloud += tmp_cloud;
    }
    LOG(INFO) << "Target frame map size: " << target_cloud->size();
    PointCloudXYZIPtr source_cloud(new PointCloudXYZI);
    int source_index = loop_pairs.source_index;
    Eigen::Affine3d source_pose = key_frames.at(source_index)->pose_;
    for(int index = source_index - near_frame_num_; index < source_index + near_frame_num_; ++index){
        if(index < 0 || index >= key_frames.size()) continue;
//        *source_cloud += *(key_frames.at(index)->corner_cloud_);
//        *source_cloud += *(key_frames.at(index)->plane_cloud_);
        tmp_cloud.clear();
        tmp_cloud += *(key_frames.at(index)->corner_cloud_);
        tmp_cloud += *(key_frames.at(index)->plane_cloud_);
        tmp_delta = source_pose.inverse() * key_frames.at(index)->pose_;
        pcl::transformPointCloud(tmp_cloud, tmp_cloud, tmp_delta);
        *source_cloud += tmp_cloud;
    }
    LOG(INFO) << "Source frame map size: " << source_cloud->size();

    Eigen::Affine3d init_delta = key_frames.at(target_index)->pose_.inverse() * key_frames.at(source_index)->pose_;
    pcl::IterativeClosestPoint<PointXYZI, PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(1);// TODO:假设误差积累不超过3m
    icp.setMaximumIterations(30);// terminate conditions
    icp.setTransformationEpsilon(1e-3);
    icp.setEuclideanFitnessEpsilon(1e-3);
    icp.setInputTarget(target_cloud);
    icp.setInputSource(source_cloud);
    PointCloudXYZI aligned_cloud;
    icp.align(aligned_cloud, init_delta.matrix().cast<float>());
    if(!icp.hasConverged() || icp.getFitnessScore() > 0.5){
        LOG(INFO) << "Fitness score: " << icp.getFitnessScore();
        return false;
    }else{
        LOG(INFO) << "Coverged with fitness score: " << icp.getFitnessScore();
        Eigen::Matrix4d final_trans = icp.getFinalTransformation().cast<double>();
        loop_pairs.trans = final_trans;
        LOG(INFO) << "Init pose: " << init_delta.matrix();
        LOG(INFO) << "Final pose: " << final_trans;

        {
            for(auto& p : target_cloud->points){
                p.intensity = 20;
            }
            for(auto& p : source_cloud->points){
                p.intensity = 230;
            }
            pcl::transformPointCloud(*source_cloud, *source_cloud, Eigen::Affine3f(final_trans.cast<float>()));
            *target_cloud += *source_cloud;
            pcl::io::savePCDFileBinaryCompressed("/home/jin/Documents/lab_slam_ws/src/lab_slam/tmp/loop/" + std::to_string(target_index)+ "_" + std::to_string(source_index) + ".pcd",
                                                 *target_cloud);
        }
        return true;
    }

}

bool LoopDetection::detect(const std::vector<KeyFramePtr> &key_frames, LoopClosurePair& closure_pair) {
    LOG(INFO) << "Start detect...";
    // detection and align
    bool closed = false;
    if(key_frames.empty()) return false;// 正常情况下不可能发生
    KeyFramePtr current_frame = key_frames.back();
    Eigen::Affine3d current_pose = current_frame->pose_;
    Eigen::Vector3d current_position(current_pose.translation());
    pcl::PointXYZ current_position_point = v2p(current_position);
    double odo_dis = current_frame->odo_dis_;
    if(!positions_->empty() && odo_dis - history_loop_odo_dis_ > 10){
        std::vector<int> near_index;
        std::vector<float> near_dis;
        positions_tree_.radiusSearch(current_position_point, 3, near_index, near_dis);// 大一点提高回环可能，但是点云前后需要拓展一定范围，从而保证点云重合度
        if(near_index.empty()){
            LOG(INFO) << "Empty index...";
            return false;
        }
        int target_index = -1;
        double max_dis = 10.0;// 积累距离至少在10m以上，找到积累距离差别最大的那个，也是时间最早的那个
        double current_dis = current_frame->odo_dis_;
        for(int index = 0; index < near_index.size(); ++index){
            double delta_dis = current_dis - key_frames.at(near_index.at(index))->odo_dis_;
            if(delta_dis > max_dis){
                max_dis = delta_dis;
                target_index = near_index.at(index);
            }
        }
        if(target_index < 0){
            LOG(INFO) << "No loop is find for frame " << key_frames.size() - 1;
        }else{
            LOG(INFO) << "*********Find loop, frame " << target_index << " vs " << key_frames.size() - 1;
            // align key frames
            closure_pair.target_index = target_index;
            closure_pair.source_index = key_frames.size() - 1;
            closed = alignKeyFrames(key_frames, closure_pair);
        }
    }

    // Add new frame
    positions_->push_back(current_position_point);
    LOG(INFO) << "Position vector size: " << positions_->size();
    positions_tree_.setInputCloud(positions_);
    if(history_loop_odo_dis_ == -1 || closed){
        history_loop_odo_dis_ = odo_dis;
    }
    return closed;
}