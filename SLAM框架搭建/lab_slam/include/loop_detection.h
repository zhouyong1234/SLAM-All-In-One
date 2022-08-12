//
// Created by jin on 2021/12/22.
//
#ifndef LAB_SLAM_LOOP_DETECTION_H
#define LAB_SLAM_LOOP_DETECTION_H

#include <glog/logging.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include "key_frame.hpp"

struct LoopClosurePair{
    int target_index;
    int source_index;
    Eigen::Affine3d trans;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

class LoopDetection{
public:
    LoopDetection();

    bool detect(const std::vector<KeyFramePtr>& key_frames, LoopClosurePair& closure_pair);

private:
    std::vector<Eigen::Affine3d> poses_;// for loop align initial
//    std::vector<Eigen::Vector3d> positions_;// for radius search
    pcl::PointCloud<pcl::PointXYZ>::Ptr positions_;// for radius search
    pcl::KdTreeFLANN<pcl::PointXYZ> positions_tree_;
    const int near_frame_num_;
    double history_loop_odo_dis_ = -1;
    bool alignKeyFrames(const std::vector<KeyFramePtr>& key_frames, LoopClosurePair& closure_pair);

};

#endif //LAB_SLAM_LOOP_DETECTION_H
