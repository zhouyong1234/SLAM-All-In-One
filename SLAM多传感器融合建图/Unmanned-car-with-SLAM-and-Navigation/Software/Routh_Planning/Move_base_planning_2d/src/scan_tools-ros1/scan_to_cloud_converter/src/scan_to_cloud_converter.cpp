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

#include "scan_to_cloud_converter/scan_to_cloud_converter.h"

#include <pcl_conversions/pcl_conversions.h>

namespace scan_tools {

ScanToCloudConverter::ScanToCloudConverter(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO("Starting ScanToCloudConverter");

  invalid_point_.x = std::numeric_limits<float>::quiet_NaN();
  invalid_point_.y = std::numeric_limits<float>::quiet_NaN();
  invalid_point_.z = std::numeric_limits<float>::quiet_NaN();

  cloud_publisher_ = nh_.advertise<PointCloudT>(
    "cloud", 1); 
  scan_subscriber_ = nh_.subscribe(
    "scan", 1, &ScanToCloudConverter::scanCallback, this);
}

ScanToCloudConverter::~ScanToCloudConverter()
{
  ROS_INFO("Destroying ScanToCloudConverter");
}

void ScanToCloudConverter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  PointCloudT::Ptr cloud_msg =
    boost::shared_ptr<PointCloudT>(new PointCloudT());

  cloud_msg->points.resize(scan_msg->ranges.size());

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    PointT& p = cloud_msg->points[i];
    float range = scan_msg->ranges[i];
    if (range > scan_msg->range_min && range < scan_msg->range_max)
    {
      float angle = scan_msg->angle_min + i*scan_msg->angle_increment;

      p.x = range * cos(angle);
      p.y = range * sin(angle);
      p.z = 0.0;
    }
    else
      p = invalid_point_;
  }

  cloud_msg->width = scan_msg->ranges.size();
  cloud_msg->height = 1;
  cloud_msg->is_dense = false; //contains nans
  pcl_conversions::toPCL(scan_msg->header, cloud_msg->header);

  cloud_publisher_.publish(cloud_msg);
}

} //namespace scan_tools
