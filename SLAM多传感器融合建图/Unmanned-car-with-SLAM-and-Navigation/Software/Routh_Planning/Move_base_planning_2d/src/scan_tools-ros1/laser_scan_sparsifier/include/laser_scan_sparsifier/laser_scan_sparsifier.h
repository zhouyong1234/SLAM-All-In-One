/*
 * Copyright (c) 2011, Ivan Dryanovski
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

#ifndef LASER_SCAN_SPARSIFIER_LASER_SCAN_SPARSIFIER_H
#define LASER_SCAN_SPARSIFIER_LASER_SCAN_SPARSIFIER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace scan_tools {

class LaserScanSparsifier
{
  public:

    LaserScanSparsifier(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~LaserScanSparsifier();

  private:

    // **** ROS-related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber scan_subscriber_;
    ros::Publisher  scan_publisher_;

    // **** paramaters

    int step_;

    // **** member functions

    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);
};

} //namespace scan_tools

#endif // LASER_SCAN_SPARSIFIER_LASER_SCAN_SPARSIFIER_H
