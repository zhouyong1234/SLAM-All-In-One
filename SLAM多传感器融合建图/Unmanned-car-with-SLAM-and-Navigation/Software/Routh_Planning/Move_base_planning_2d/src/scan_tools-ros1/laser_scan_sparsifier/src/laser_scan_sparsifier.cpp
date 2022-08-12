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

#include "laser_scan_sparsifier/laser_scan_sparsifier.h"

namespace scan_tools {

LaserScanSparsifier::LaserScanSparsifier(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO ("Starting LaserScanSparsifier");

  // **** get paramters

  if (!nh_private_.getParam ("step", step_))
    step_ = 2;

  ROS_ASSERT_MSG(step_ > 0,
    "step parameter is set to %, must be > 0", step_);

  // **** advertise topics

  scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(
    "scan_sparse", 10);

  // **** subscribe to laser scan messages

  scan_subscriber_ = nh_.subscribe(
    "scan", 10, &LaserScanSparsifier::scanCallback, this);
}

LaserScanSparsifier::~LaserScanSparsifier ()
{
  ROS_INFO ("Destroying LaserScanSparsifier");
}

void LaserScanSparsifier::scanCallback (const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  sensor_msgs::LaserScan::Ptr scan_sparse;
  scan_sparse = boost::make_shared<sensor_msgs::LaserScan>();

  // copy over equal fields

  scan_sparse->header          = scan_msg->header;
  scan_sparse->range_min       = scan_msg->range_min;
  scan_sparse->range_max       = scan_msg->range_max;
  scan_sparse->angle_min       = scan_msg->angle_min;
  scan_sparse->angle_increment = scan_msg->angle_increment * step_;
  scan_sparse->time_increment  = scan_msg->time_increment;
  scan_sparse->scan_time       = scan_msg->scan_time;

  // determine size of new scan

  unsigned int size_sparse = scan_msg->ranges.size() / step_;
  scan_sparse->ranges.resize(size_sparse);

  // determine new maximum angle

  scan_sparse->angle_max = 
    scan_sparse->angle_min + (scan_sparse->angle_increment * (size_sparse - 1));

  for (unsigned int i = 0; i < size_sparse; i++)
  {
    scan_sparse->ranges[i] = scan_msg->ranges[i * step_];
    // TODO - also copy intensity values
  }

  scan_publisher_.publish(scan_sparse);
}

} //namespace scan_tools

