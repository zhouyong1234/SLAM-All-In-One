//
// Created by jin on 2021/5/19.
//

#ifndef LABLOAM_FEATURES_REGISTER_H
#define LABLOAM_FEATURES_REGISTER_H

#include "utility.hpp"
#include "cost_function.hpp"
#include <glog/logging.h>
class FeaturesRegister{
public:
    FeaturesRegister(int max_ite, float distance_threshold);
    void setInitial(double const *delta);
    void setInitial(const Eigen::Affine3d& initial_pose);

    virtual bool align(PointCloudXYZIPtr corner_map, PointCloudXYZIPtr plane_map,
               PointCloudXYZIPtr corner_cloud, PointCloudXYZIPtr plane_cloud, Eigen::Affine3d initial_pose) = 0;

    Eigen::Affine3d getTransform(){
        return Eigen::Affine3d(Eigen::Translation3d(t_) * q_);
    }

public:
    double delta_q_[4] = {0.0, 0.0, 0.0, 1.0};// x, y, z, w，这是ceres所决定的。而在Eigen::Quaternion的初始化中，w放在前面
    double delta_t_[3] = {0.0, 0.0, 0.0};
    Eigen::Map<Eigen::Quaterniond> q_ = Eigen::Map<Eigen::Quaterniond>(delta_q_);
    Eigen::Map<Eigen::Vector3d> t_ = Eigen::Map<Eigen::Vector3d>(delta_t_);
    int max_ite_ = 8;// TODO
    float nn_distance_threshold_ = 2;// 0.5
    pcl::KdTreeFLANN<PointXYZI> target_corner_tree_;
    pcl::KdTreeFLANN<PointXYZI> target_plane_tree_;
};

class Scan2ScanMatcher : public FeaturesRegister{
public:
    Scan2ScanMatcher(int max_ite, float distance_threshold, bool deskew, int near_line);

    // 如果不需要考虑运动畸变，那么可以认为点都是在最后时刻瞬间完成的，因此只需要考虑delta_pose，就可以投影到起始时刻（也是上一帧的终止时刻）
    // 如果需要考虑畸变，那么delta_pose的过程中始终存在畸变，因此对delta_pose进行插值后再投影
    // 最后的结果都是基于delta_pose,将本帧的点在上一帧坐标系下表示，从而确定最近邻
    void pointProjToStart(const PointXYZI& local_point, PointXYZI& result_point);

    void cloudProjToStart(PointCloudXYZIPtr local_cloud, PointCloudXYZIPtr return_cloud){
        if(return_cloud != local_cloud){
            return_cloud->resize(local_cloud->size());
        }
        for(int index = 0; index < local_cloud->size(); ++index){
            pointProjToStart(local_cloud->points[index], return_cloud->points[index]);
        }
        assert(local_cloud->points.size() == return_cloud->points.size());
    }

    void cloudProjToEnd(PointCloudXYZIPtr local_cloud){
        cloudProjToStart(local_cloud, local_cloud);
        pcl::transformPointCloud(*local_cloud, *local_cloud, Eigen::Affine3d(Eigen::Translation3d(t_) * q_).inverse());
    }

    virtual bool align(PointCloudXYZIPtr corner_map, PointCloudXYZIPtr plane_map,
        PointCloudXYZIPtr corner_cloud, PointCloudXYZIPtr plane_cloud, Eigen::Affine3d initial_pose);

private:
    bool deskew_ = false;
    int NEAR_LINE_NUM = 4;
};

class Scan2MapMatcher : public FeaturesRegister{
public:
    Scan2MapMatcher(int max_ite, float distance_threshold);
    virtual bool align(PointCloudXYZIPtr corner_map, PointCloudXYZIPtr plane_map,
                       PointCloudXYZIPtr corner_cloud, PointCloudXYZIPtr plane_cloud, Eigen::Affine3d initial_pose);
    void pointProject(const PointXYZI& local_point, PointXYZI& result_point);
private:
    int near_corners_num_ = 10;
    int near_planes_num_ = 20;
};
#endif //LABLOAM_FEATURES_REGISTER_H
