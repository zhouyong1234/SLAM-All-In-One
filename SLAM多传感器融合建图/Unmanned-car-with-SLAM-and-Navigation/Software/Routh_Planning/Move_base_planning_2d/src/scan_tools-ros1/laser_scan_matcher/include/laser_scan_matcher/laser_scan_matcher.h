/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

#ifndef LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
#define LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min
#undef max

namespace scan_tools
{

class LaserScanMatcher
{
  public:

    LaserScanMatcher(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~LaserScanMatcher();

  private:

    typedef pcl::PointXYZ           PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    // **** ros

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber scan_subscriber_;
    ros::Subscriber cloud_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Subscriber vel_subscriber_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    tf::Transform base_to_laser_; // static, cached
    tf::Transform laser_to_base_; // static, cached, calculated from base_to_laser_

    ros::Publisher  pose_publisher_;
    ros::Publisher  pose_stamped_publisher_;
    ros::Publisher  pose_with_covariance_publisher_;
    ros::Publisher  pose_with_covariance_stamped_publisher_;

    // **** parameters

    std::string base_frame_;
    std::string fixed_frame_;
    double cloud_range_min_;
    double cloud_range_max_;
    double cloud_res_;
    bool publish_tf_;
    bool publish_pose_;
    bool publish_pose_with_covariance_;
    bool publish_pose_stamped_;
    bool publish_pose_with_covariance_stamped_;
    std::vector<double> position_covariance_;
    std::vector<double> orientation_covariance_;

    bool use_cloud_input_;

    double kf_dist_linear_;
    double kf_dist_linear_sq_;
    double kf_dist_angular_;

    // **** What predictions are available to speed up the ICP?
    // 1) imu - [theta] from imu yaw angle - /imu topic
    // 2) odom - [x, y, theta] from wheel odometry - /odom topic
    // 3) velocity [vx, vy, vtheta], usually from ab-filter - /vel.
    // If more than one is enabled, priority is imu > odom > velocity

    bool use_imu_;
    bool use_odom_;
    bool use_vel_;
    bool stamped_vel_;

    // **** state variables

    boost::mutex mutex_;

    bool initialized_;
    bool received_imu_;
    bool received_odom_;
    bool received_vel_;

    tf::Transform f2b_;    // fixed-to-base tf (pose of base frame in fixed frame)
    tf::Transform f2b_kf_; // pose of the last keyframe scan in fixed frame

    ros::Time last_icp_time_;

    sensor_msgs::Imu latest_imu_msg_;
    sensor_msgs::Imu last_used_imu_msg_;
    nav_msgs::Odometry latest_odom_msg_;
    nav_msgs::Odometry last_used_odom_msg_;

    geometry_msgs::Twist latest_vel_msg_;

    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;

    // **** methods

    void initParams();
    void processScan(LDP& curr_ldp_scan, const ros::Time& time);

    void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                              LDP& ldp);
    void PointCloudToLDP(const PointCloudT::ConstPtr& cloud,
                               LDP& ldp);

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void cloudCallback (const PointCloudT::ConstPtr& cloud);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg);
    void velCallback (const geometry_msgs::Twist::ConstPtr& twist_msg);
    void velStmpCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg);

    void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    bool getBaseToLaserTf (const std::string& frame_id);

    bool newKeyframeNeeded(const tf::Transform& d);

    void getPrediction(double& pr_ch_x, double& pr_ch_y,
                       double& pr_ch_a, double dt);

    void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);
};

} // namespace scan_tools

#endif // LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
