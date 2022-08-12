/* 
 * scout_messenger.hpp
 * 
 * Created on: Jun 14, 2019 10:24
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_MESSENGER_HPP
#define SCOUT_MESSENGER_HPP

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// #include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "scout_msgs/ScoutLightCmd.h"
#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include <mutex>

namespace westonrobot
{
class ScoutROSMessenger
{
public:
    explicit ScoutROSMessenger(ros::NodeHandle *nh);
    ScoutROSMessenger(ScoutRobot *scout, ros::NodeHandle *nh);

    std::string odom_frame_;
    std::string base_frame_;
    std::string odom_topic_name_;
    bool pub_tf;

    bool simulated_robot_ = false;
    int sim_control_rate_ = 50;

    void SetupSubscription();

    void PublishStateToROS();
    void PublishSimStateToROS(double linear, double angular);

    void GetCurrentMotionCmdForSim(double &linear, double &angular);

private:
    ScoutRobot *scout_;
    ros::NodeHandle *nh_;

    std::mutex twist_mutex_;
    geometry_msgs::Twist current_twist_;

    ros::Publisher odom_publisher_;
    ros::Publisher status_publisher_;
    ros::Publisher BMS_status_publisher_;
    ros::Subscriber motion_cmd_subscriber_;
    ros::Subscriber light_cmd_subscriber_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // speed variables
    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    ros::Time last_time_;
    ros::Time current_time_;

    void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void LightCmdCallback(const scout_msgs::ScoutLightCmd::ConstPtr &msg);
    void PublishOdometryToROS(double linear, double angular, double dt);
};
} // namespace westonrobot

#endif /* SCOUT_MESSENGER_HPP */
