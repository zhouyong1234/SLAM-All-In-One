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

#include "laser_scan_splitter/laser_scan_splitter.h"

namespace scan_tools {

LaserScanSplitter::LaserScanSplitter(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO ("Starting LaserScanSplitter");

  // **** get paramters

  std::string topics_string;
  std::string frames_string;
  std::string sizes_string;

  if (!nh_private_.getParam ("topics", topics_string))
    topics_string = "scan1 scan2";
  if (!nh_private_.getParam ("frames", frames_string))
    frames_string = "laser laser";
  if (!nh_private_.getParam ("sizes", sizes_string))
    sizes_string = "256 256";

  // **** tokenize inputs
  tokenize (topics_string, published_scan_topics_);
  tokenize (frames_string, published_laser_frames_);

  std::vector<std::string> sizes_tokens;
  tokenize (sizes_string, sizes_tokens);

  size_sum_ = 0;
  for (unsigned int i = 0; i < sizes_tokens.size (); i++)
  {
    sizes_.push_back (atoi (sizes_tokens[i].c_str ()));
    size_sum_ += sizes_[i];

    ROS_ASSERT_MSG ((sizes_[i] > 0), "LaserScanSplitter: Scan size cannot be zero. Quitting.");
  }

  // **** check that topic, frames, and sizes vectors have same sizes

  ROS_ASSERT_MSG ((published_scan_topics_.size () == published_laser_frames_.size ()) &&
                  (sizes_.size () == published_laser_frames_.size ()),
                  "LaserScanSplitter: Invalid parameters. Quitting.");

  // **** subscribe to laser scan messages
  scan_subscriber_ = nh_.subscribe (scan_topic_, 10, &LaserScanSplitter::scanCallback, this);

  // **** advertise topics
  for (unsigned int i = 0; i < published_scan_topics_.size (); i++)
  {
    scan_publishers_.push_back (ros::Publisher ());
    scan_publishers_[i] = 
      nh_.advertise<sensor_msgs::LaserScan>(published_scan_topics_[i], 10);
  }
}

LaserScanSplitter::~LaserScanSplitter ()
{
  ROS_INFO ("Destroying LaserScanSplitter");
}

void LaserScanSplitter::scanCallback (const sensor_msgs::LaserScanConstPtr & scan_msg)
{
  // **** check for scan size
  if (size_sum_ != scan_msg->ranges.size ())
  {
    ROS_WARN ("LaserScanSplitter: Received a laser scan with size (%d) \
      incompatible to input parameters. Skipping scan.", (int)scan_msg->ranges.size());
    return;
  }

  // **** copy information over
  int r = 0;

  for (unsigned int i = 0; i < published_scan_topics_.size (); i++)
  {
    sensor_msgs::LaserScan::Ptr scan_segment;
    scan_segment = boost::make_shared<sensor_msgs::LaserScan>();

    scan_segment->header = scan_msg->header;
    scan_segment->range_min = scan_msg->range_min;
    scan_segment->range_max = scan_msg->range_max;
    scan_segment->angle_increment = scan_msg->angle_increment;
    scan_segment->time_increment = scan_msg->time_increment;
    scan_segment->scan_time = scan_msg->scan_time;
    scan_segment->header.frame_id = published_laser_frames_[i];

    scan_segment->angle_min = 
      scan_msg->angle_min + (scan_msg->angle_increment * r);
    scan_segment->angle_max = 
      scan_msg->angle_min + (scan_msg->angle_increment * (r + sizes_[i] - 1));

    // TODO - also copy intensity values

    scan_segment->ranges.resize(sizes_[i]);
    memcpy(&scan_segment->ranges[0], &scan_msg->ranges[r], sizes_[i]*sizeof(int));
    r+=sizes_[i];

    scan_publishers_[i].publish (scan_segment);
  }
}

void LaserScanSplitter::tokenize (const std::string & str, std::vector < std::string > &tokens)
{
  std::string::size_type last_pos = str.find_first_not_of (" ", 0);
  std::string::size_type pos = str.find_first_of (" ", last_pos);

  while (std::string::npos != pos || std::string::npos != last_pos)
  {
    std::string string_token = str.substr (last_pos, pos - last_pos);
    tokens.push_back (string_token);
    last_pos = str.find_first_not_of (" ", pos);
    pos = str.find_first_of (" ", last_pos);
  }
}

} //namespace scan_tools

