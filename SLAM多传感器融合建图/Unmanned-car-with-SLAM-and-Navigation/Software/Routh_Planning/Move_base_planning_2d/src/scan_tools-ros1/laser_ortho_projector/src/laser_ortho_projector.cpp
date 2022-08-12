/*
 * Copyright (c) 2010, 2011, Ivan Dryanovski, William Morris
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

#include "laser_ortho_projector/laser_ortho_projector.h"

#include <pcl_conversions/pcl_conversions.h>

namespace scan_tools {

LaserOrthoProjector::LaserOrthoProjector (ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO ("Starting LaserOrthoProjector");

  initialized_ = false;

  // set initial orientation to 0

  ortho_to_laser_.setIdentity();

  nan_point_.x = std::numeric_limits<float>::quiet_NaN();
  nan_point_.y = std::numeric_limits<float>::quiet_NaN();
  nan_point_.z = std::numeric_limits<float>::quiet_NaN();

  // **** parameters

  if (!nh_private_.getParam ("fixed_frame", world_frame_))
    world_frame_ = "/world";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/base_link";
  if (!nh_private_.getParam ("ortho_frame", ortho_frame_))
    ortho_frame_ = "/base_ortho";
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = false;
  if (!nh_private_.getParam ("use_pose", use_pose_))
    use_pose_ = true;
  if (!nh_private_.getParam ("use_imu", use_imu_))
    use_imu_ = false;

  if (use_imu_ && use_pose_)
    ROS_FATAL("use_imu and use_pose params cannot both be true");
  if (!use_imu_ && !use_pose_)
    ROS_FATAL("use_imu and use_pose params cannot both be false");


  // **** subscribe to laser scan messages

  scan_subscriber_ = nh_.subscribe(
    "scan", 10, &LaserOrthoProjector::scanCallback, this);

  if (use_pose_)
  {
    pose_subscriber_ = nh_.subscribe(
      "pose", 10, &LaserOrthoProjector::poseCallback, this);
  }
  if (use_imu_)
  {
    imu_subscriber_ = nh_.subscribe(
      "imu/data", 10, &LaserOrthoProjector::imuCallback, this);
  }

  // **** advertise orthogonal scan

  cloud_publisher_ = nh_.advertise<PointCloudT>(
    "cloud_ortho", 10);
}

LaserOrthoProjector::~LaserOrthoProjector()
{

}

void LaserOrthoProjector::imuCallback(const ImuMsg::ConstPtr& imu_msg)
{
  // obtain world to base frame transform from the pose message
  tf::Transform world_to_base;
  world_to_base.setIdentity();
  
  tf::Quaternion q;
  tf::quaternionMsgToTF(imu_msg->orientation, q);
  world_to_base.setRotation(q);

  // calculate world to ortho frame transform
  tf::Transform world_to_ortho;
  getOrthoTf(world_to_base, world_to_ortho);
  
  if (publish_tf_)
  {
    tf::StampedTransform world_to_ortho_tf(
      world_to_ortho, imu_msg->header.stamp, world_frame_, ortho_frame_);
    tf_broadcaster_.sendTransform(world_to_ortho_tf);
  }

  // calculate ortho to laser tf, and save it for when scans arrive
  ortho_to_laser_ = world_to_ortho.inverse() * world_to_base * base_to_laser_;
}

void LaserOrthoProjector::poseCallback(const PoseMsg::ConstPtr& pose_msg)
{
  // obtain world to base frame transform from the pose message
  tf::Transform world_to_base;
  tf::poseMsgToTF(pose_msg->pose, world_to_base);   

  // calculate world to ortho frame transform
  tf::Transform world_to_ortho;
  getOrthoTf(world_to_base, world_to_ortho);
  
  if (publish_tf_)
  {
    tf::StampedTransform world_to_ortho_tf(
      world_to_ortho, pose_msg->header.stamp, world_frame_, ortho_frame_);
    tf_broadcaster_.sendTransform(world_to_ortho_tf);
  }

  // calculate ortho to laser tf, and save it for when scans arrive
  ortho_to_laser_ = world_to_ortho.inverse() * world_to_base * base_to_laser_;
}

void LaserOrthoProjector::getOrthoTf(const tf::Transform& world_to_base, tf::Transform& world_to_ortho)
{
  const tf::Vector3&    w2b_o = world_to_base.getOrigin();
  const tf::Quaternion& w2b_q = world_to_base.getRotation();

  tf::Vector3 wto_o(w2b_o.getX(), w2b_o.getY(), 0.0);
  tf::Quaternion wto_q = tf::createQuaternionFromYaw(tf::getYaw(w2b_q));
  
  world_to_ortho.setOrigin(wto_o);
  world_to_ortho.setRotation(wto_q);
} 

void LaserOrthoProjector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  if(!initialized_)
  {
    initialized_ = getBaseToLaserTf(scan_msg);

    if (initialized_) createCache(scan_msg);
    else return;
  }

  if(!use_pose_)
  {
    // obtain transform between fixed and base frame
    tf::StampedTransform world_to_base_tf;
    try
    {
      tf_listener_.waitForTransform (
        world_frame_, base_frame_, scan_msg->header.stamp, ros::Duration(0.1));
      tf_listener_.lookupTransform (
        world_frame_, base_frame_, scan_msg->header.stamp, world_to_base_tf);
    }
    catch (tf::TransformException ex)
    {
      // transform unavailable - skip scan
      ROS_WARN("Skipping scan %s", ex.what());
      return;
    }

    // calculate world to ortho frame transform
    tf::Transform world_to_ortho;
    getOrthoTf(world_to_base_tf, world_to_ortho);
  }

  // **** build and publish projected cloud

  PointCloudT::Ptr cloud = 
    boost::shared_ptr<PointCloudT>(new PointCloudT());

  pcl_conversions::toPCL(scan_msg->header, cloud->header);
  cloud->header.frame_id = ortho_frame_;

  for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
  {
    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min)
    {
      tf::Vector3 p(r * a_cos_[i], r * a_sin_[i], 0.0);
      p = ortho_to_laser_ * p;

      PointT point;
      point.x = p.getX();
      point.y = p.getY();
      point.z = 0.0;
      cloud->points.push_back(point);
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true; // no nan's present 

  cloud_publisher_.publish (cloud);
}

bool LaserOrthoProjector::getBaseToLaserTf (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener_.waitForTransform(
      base_frame_, scan_msg->header.frame_id, scan_msg->header.stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, scan_msg->header.frame_id, scan_msg->header.stamp, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("LaserOrthoProjector: Could not get initial laser transform(%s)", ex.what());
    return false;
  }
  base_to_laser_ = base_to_laser_tf;

  return true;
}

void LaserOrthoProjector::createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }
}

} //namespace scan_tools
