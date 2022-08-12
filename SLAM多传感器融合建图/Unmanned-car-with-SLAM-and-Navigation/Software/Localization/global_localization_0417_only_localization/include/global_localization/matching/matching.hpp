#ifndef GLOBAL_LOCALIZATION_MATCHING_MATCHING_HPP_
#define GLOBAL_LOCALIZATION_MATCHING_MATCHING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "global_localization/sensor_data/cloud_data.hpp"
#include "global_localization/sensor_data/pose_data.hpp"

#include "global_localization/scan_context_manager/scan_context_manager.hpp"
#include "global_localization/registration/registration_interface.hpp"
#include "global_localization/cloud_filter/cloud_filter_interface.hpp"
#include "global_localization/cloud_filter/box_filter.hpp"
// subscriber
#include "global_localization/subscriber/cloud_subscriber.hpp"
#include "global_localization/subscriber/odometry_subscriber.hpp"
// publisher
#include "global_localization/publisher/cloud_publisher.hpp"
#include "global_localization/publisher/odometry_publisher.hpp"
#include "global_localization/publisher/tf_broadcaster.hpp"

namespace global_localization {
class Matching {
  public:
    //构造函数，用于初始化
    Matching(ros::NodeHandle& nh);
    //地图读取
    void GetGlobalMap(CloudData::CLOUD_PTR& global_map);
    bool HasNewGlobalMap();
    bool HasNewLocalMap();
    CloudData::CLOUD_PTR& GetLocalMap();

    //实时数据读取后进行定位
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateMatching();
    bool PublishData();

  private:
    //初始化
    bool InitWithConfig();
    bool InitGlobalMap();
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool ResetLocalMap(float x, float y, float z);
    bool InitBoxFilter(const YAML::Node& config_node);
    bool InitScanContextManager(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool HasInited();
    bool Init_sub_pub(ros::NodeHandle& nh);
    bool SetInited(void);

    //点云匹配
    bool SetGNSSPose(const Eigen::Matrix4f& init_pose);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool SetScanContextPose(const CloudData& init_scan);
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
    CloudData::CLOUD_PTR& GetCurrentScan();

  private:

    std::string scan_context_path_ = "";
    std::string map_path_ = "";
    std::string loop_closure_method_ = "";

    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;

    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_; 
    std::shared_ptr<BoxFilter> box_filter_ptr_;

    std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;

    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

    // subscriber 
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;

    std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<TFBroadCaster> laser_tf_pub_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> gnss_data_buff_;

    CloudData current_cloud_data_;
    PoseData current_gnss_data_;

    bool has_inited_ = false;
    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;
};
}

#endif