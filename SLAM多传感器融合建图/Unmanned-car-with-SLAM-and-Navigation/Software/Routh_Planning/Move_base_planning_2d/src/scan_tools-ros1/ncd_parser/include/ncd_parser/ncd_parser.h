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

#ifndef NCD_PARSER_NCD_PARSER
#define NCD_PARSER_NCD_PARSER

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

const double DEG_TO_RAD = 3.14159 / 180.0;

const double RANGE_MIN = 0.20;
const double RANGE_MAX = 50.0;

const std::string worldFrame_      = "map";
const std::string odomFrame_       = "odom";
const std::string leftLaserFrame_  = "laser_left";
const std::string rightLaserFrame_ = "laser_right";

class NCDParser
{
  private:

    char* filename_;
    double rate_;
    double start_, end_;
    double lastTime_;

    ros::Publisher  leftLaserPublisher_;
    ros::Publisher  rightLaserPublisher_;

    tf::TransformBroadcaster tfBroadcaster_;

    tf::Transform worldToOdom_;
    tf::Transform  odomToLeftLaser_;
    tf::Transform  odomToRightLaser_;

    void publishLaserMessage(const std::vector<std::string>& tokens,
                             const std::string& laserFrame,
                             const ros::Publisher& publisher);

    void publishTfMessages(const std::vector<std::string>& tokens);

    void tokenize (const std::string& str, 
                         std::vector <std::string> &tokens, 
                         std::string sentinel);

    std::vector<float> extractArray(std::string s, std::string pattern);

    double extractValue(std::string s, std::string pattern);

    void createOdomToLeftLaserTf();
    void createOdomToRightLaserTf();
  
  public:

    NCDParser(char* filename);
    virtual ~NCDParser();

    void launch();
};

#endif
