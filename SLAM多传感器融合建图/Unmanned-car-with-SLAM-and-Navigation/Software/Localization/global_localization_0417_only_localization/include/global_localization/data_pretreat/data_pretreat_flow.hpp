#ifndef GLOBAL_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define GLOBAL_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "global_localization/subscriber/cloud_subscriber.hpp"
#include "global_localization/subscriber/imu_subscriber.hpp"
#include "global_localization/subscriber/velocity_subscriber.hpp"
#include "global_localization/subscriber/gnss_subscriber.hpp"
#include "global_localization/tf_listener/tf_listener.hpp"
// publisher
#include "global_localization/publisher/cloud_publisher.hpp"
#include "global_localization/publisher/odometry_publisher.hpp"
// models
#include "global_localization/scan_adjust/distortion_adjust.hpp"

namespace global_localization {
class DataPretreatFlow {
  public:
    DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic);

    bool Run();

  private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    // models
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    VelocityData current_velocity_data_;
    GNSSData current_gnss_data_;

    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}

#endif