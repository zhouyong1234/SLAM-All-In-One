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

#ifndef SCAN_TO_CLOUD_CONVERTER_SCAN_TO_CLOUD_CONVERTER_H
#define SCAN_TO_CLOUD_CONVERTER_SCAN_TO_CLOUD_CONVERTER_H

#include <limits>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

namespace scan_tools {

class ScanToCloudConverter
{
  typedef pcl::PointXYZ           PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher cloud_publisher_;
    ros::Subscriber scan_subscriber_;

    PointT invalid_point_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
 
 public:

    ScanToCloudConverter(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~ScanToCloudConverter();
};

} // namespace scan_tools

#endif // SCAN_TO_CLOUD_CONVERTER_LASER_TO_CLOUD_CONVERTER_H
