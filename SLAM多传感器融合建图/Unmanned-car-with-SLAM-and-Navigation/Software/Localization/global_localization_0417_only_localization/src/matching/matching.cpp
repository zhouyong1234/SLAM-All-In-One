#include "global_localization/matching/matching.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "global_localization/global_defination/global_defination.h"
#include "global_localization/registration/ndt_registration.hpp"
#include "global_localization/cloud_filter/voxel_filter.hpp"
#include "global_localization/cloud_filter/no_filter.hpp"

namespace global_localization {
Matching::Matching(ros::NodeHandle& nh)
: global_map_ptr_(new CloudData::CLOUD()),
  local_map_ptr_(new CloudData::CLOUD()),
  current_scan_ptr_(new CloudData::CLOUD()) 

{

    InitWithConfig();   //初始化yaml

    Init_sub_pub(nh);   //初始化订阅话题

    InitGlobalMap();    //初始化全局地图

    ResetLocalMap(0.0, 0.0, 0.0);   //初始化全局子地图

}
bool Matching::Init_sub_pub(ros::NodeHandle& nh) {

    // subscriber:
    // a. undistorted Velodyne measurement: 
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/velodyne_points", 100000);

    // b. lidar pose in map frame:
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

    // publisher:
    // a. global point cloud map:
    // global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    // b. local point cloud map:
    // local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    // c. current scan:
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
    // d. estimated lidar pose in map frame:
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/map", "/lidar", 100);
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/vehicle_link");
    return true;
}

bool Matching::InitWithConfig() {
    LOG(INFO) << std::endl
              << "-----------------Init Localization-------------------" 
              << std::endl;

    std::string config_file_path = WORK_SPACE_PATH + "/config/params.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    map_path_ = config_node["map_path"].as<std::string>();

    // a. global map filter -- downsample point cloud map for visualization:
    InitFilter("global_map", global_map_filter_ptr_, config_node);
    // b. local map filter -- downsample & ROI filtering for scan-map matching:
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    // c. scan filter -- 
    InitFilter("frame", frame_filter_ptr_, config_node);

    InitBoxFilter(config_node);
    InitRegistration(registration_ptr_, config_node);

    InitScanContextManager(config_node);
    return true;
}

bool Matching::InitGlobalMap() {
    pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
    LOG(INFO) << "Load global map, size:" << global_map_ptr_->points.size();

    // since scan-map matching is used, here apply the same filter to local map & scan:
    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
    LOG(INFO) << "Filtered global map, size:" << global_map_ptr_->points.size();
    
    has_new_global_map_ = true;
    return true;
}

bool Matching::ResetLocalMap(float x, float y, float z) {
    std::vector<float> origin = {x, y, z};

    // use ROI filtering for local map segmentation:
    box_filter_ptr_->SetOrigin(origin);
    box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

    registration_ptr_->SetInputTarget(local_map_ptr_);

    has_new_local_map_ = true;

    std::vector<float> edge = box_filter_ptr_->GetEdge();
    LOG(INFO) << "New local map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl << std::endl;

    return true;
}

bool Matching::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "\tFilter Method for " << filter_user << ": " << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "Filter method " << filter_mothod << " for " << filter_user << " NOT FOUND!";
        return false;
    }
    return true;
}

bool Matching::InitBoxFilter(const YAML::Node& config_node) {
    box_filter_ptr_ = std::make_shared<BoxFilter>(config_node);
    return true;
}

bool Matching::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "\tPoint Cloud Registration Method: " << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "Registration method " << registration_method << " NOT FOUND!";
        return false;
    }

    return true;
}

bool Matching::InitScanContextManager(const YAML::Node& config_node) {
    // get loop closure config:
    loop_closure_method_ = config_node["loop_closure_method"].as<std::string>();

    // create instance:
    scan_context_manager_ptr_ = std::make_shared<ScanContextManager>(config_node[loop_closure_method_]);

    // load pre-built index:
    scan_context_path_ = config_node["scan_context_path"].as<std::string>();
    scan_context_manager_ptr_->Load(scan_context_path_);

    return true;
}

void Matching::GetGlobalMap(CloudData::CLOUD_PTR& global_map) {
    // downsample global map for visualization:
    global_map_filter_ptr_->Filter(global_map_ptr_, global_map);

    has_new_global_map_ = false;
}

CloudData::CLOUD_PTR& Matching::GetLocalMap() {
    return local_map_ptr_;
}

CloudData::CLOUD_PTR& Matching::GetCurrentScan() {
    return current_scan_ptr_;
}

bool Matching::HasNewGlobalMap() {
    return has_new_global_map_;
}

bool Matching::HasNewLocalMap() {
    return has_new_local_map_;
}

bool Matching::HasInited() {
    return has_inited_;
}

bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose) {                              // 设置定位的初始位姿，根据此位姿可以找到定位需要用到的局部地图；这个位姿可以通过GNSS数据得到，或者回环检测得到
    init_pose_ = init_pose;
    ResetLocalMap(init_pose(0,3), init_pose(1,3), init_pose(2,3));                 //  定位需要用到的局部地图，通过获取(x,y,z) 更新local map

    return true;
}

bool Matching::SetInited(void) {
    has_inited_ = true;

    return true;
}

bool Matching::SetGNSSPose(const Eigen::Matrix4f& gnss_pose) {      //   利用GNSS 数据找到定位的初始位姿，初始定位的GNSS值，需要在建图时保存
    static int gnss_cnt = 0;

    current_gnss_pose_ = gnss_pose;

    if ( gnss_cnt == 0 ) {
        SetInitPose(gnss_pose);
    } else if (gnss_cnt > 3) {
        has_inited_ = true;
    }

    gnss_cnt++;

    return true;
}

bool Matching::SetScanContextPose(const CloudData& init_scan) {                      //    利用闭环检测，找到定位的初始位姿
    // get init pose proposal using scan context match:
    Eigen::Matrix4f init_pose =  Eigen::Matrix4f::Identity();                                             //    初始化位姿为单位阵

    if (
        !scan_context_manager_ptr_->DetectLoopClosure(init_scan, init_pose)
    ) {
        return false;
    }

    // set init pose:
    SetInitPose(init_pose);
    has_inited_ = true;
    
    return true;
}

bool Matching::ReadData() {
    // pipe lidar measurements and pose into buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    return true;
}

bool Matching::HasData() {
    // if (cloud_data_buff_.size() > 0)
    //{
            // CloudData cloud_data = cloud_data_buff_.front();  //存到buffer
            // current_scan_pub_ptr_->Publish(cloud_data.cloud_ptr);
            // cloud_data_buff_.pop_front();
    //}
    if (cloud_data_buff_.size() == 0)
        return false;
    
    if (HasInited())
        return true;
    
    if (gnss_data_buff_.size() == 0)
        return false;
        
    return true;
}

bool Matching::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();

    if (HasInited()) {
        cloud_data_buff_.pop_front();
        gnss_data_buff_.clear();
        return true;
    }

    current_gnss_data_ = gnss_data_buff_.front();

    double diff_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}
bool Matching::UpdateMatching() {
    if (!HasInited()) {                //  第一帧点云数据

        /*地图原点初始化    初始化设置 init_pose  为单位阵*/  
        // Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();          
        // SetInitPose(init_pose);
        // SetInited();
        
        /*利用ScanContext 进行位姿初始化*/
        SetScanContextPose(current_cloud_data_);
        /*利用GNSS 进行位姿初始化*/
        //SetGNSSPose(current_gnss_data_.pose);
    }
    return Update(current_cloud_data_, laser_odometry_);

}

bool Matching::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;

    // remove invalid measurements:
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, indices);

    // downsample:
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_cloud_ptr);

    if (!has_inited_) {
        predict_pose = current_gnss_pose_;
    }

    // matching:
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, cloud_pose);
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *current_scan_ptr_, cloud_pose);

    // update predicted pose:
    step_pose = last_pose.inverse() * cloud_pose;
    predict_pose = cloud_pose * step_pose;
    last_pose = cloud_pose;

    // 匹配之后判断是否需要更新局部地图
    std::vector<float> edge = box_filter_ptr_->GetEdge();
    for (int i = 0; i < 3; i++) {
        if (
            fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
            fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0
        ) {
            continue;
        }
            
        ResetLocalMap(cloud_pose(0,3), cloud_pose(1,3), cloud_pose(2,3));
        break;
    }
    return true;
}

bool Matching::PublishData() {
    laser_tf_pub_ptr_->SendTransform(laser_odometry_, current_cloud_data_.time);
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);
    current_scan_pub_ptr_->Publish(GetCurrentScan());
    return true;
}
}