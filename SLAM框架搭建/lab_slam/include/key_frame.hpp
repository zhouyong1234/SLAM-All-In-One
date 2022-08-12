#ifndef KEY_FRAME_HPP
#define KEY_FRAME_HPP

#include "utility.hpp"

struct KeyFrame{
public:
    KeyFrame(int index, Eigen::Affine3d odo_pose, double accumulate_dis, Eigen::Affine3d pose, PointCloudXYZIPtr corners, PointCloudXYZIPtr planes) : odo_pose_(odo_pose){
        index_ = index;
        pose_ = pose;
        odo_dis_ = accumulate_dis;
        corner_cloud_ = std::move(corners);
        plane_cloud_ = std::move(planes);
    }

//    PointCloudXYZIPtr getGlobalCornerCloud(){
//        return std::make_shared<PointCloudXYZI>(pcl::trans)
//    }
//
//    PointCloudXYZIPtr getGlobalPlaneCloud(){
//
//    }
    int index_;
    const Eigen::Affine3d odo_pose_;
    Eigen::Affine3d pose_;// optimal odometry pose
    double odo_dis_;// 从第一个关键帧起，积累的里程计距离
    PointCloudXYZIPtr corner_cloud_ = nullptr;
    PointCloudXYZIPtr plane_cloud_ = nullptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

typedef std::shared_ptr<KeyFrame> KeyFramePtr;
#endif