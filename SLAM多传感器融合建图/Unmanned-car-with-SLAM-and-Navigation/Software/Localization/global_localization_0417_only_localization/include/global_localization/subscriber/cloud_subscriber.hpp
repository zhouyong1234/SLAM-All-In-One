#ifndef GLOBAL_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define GLOBAL_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


#include "global_localization/sensor_data/cloud_data.hpp"

namespace global_localization {
class CloudSubscriber {
  public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    CloudSubscriber() = default;
    void ParseData(std::deque<CloudData>& deque_cloud_data);

  private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<CloudData> new_cloud_data_;

    std::mutex buff_mutex_;
};
}

#endif