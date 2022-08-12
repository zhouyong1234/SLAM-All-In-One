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

#include "ncd_parser/ncd_parser.h"

int main(int argc, char** argv)
{
  if(argc != 4)
  {
    printf("usage is `rosrun ncd_parser ncd_parser filename.alog`\n");
    return 1;
  }

  ros::init (argc, argv, "ncd_parser");
  NCDParser parser(argv[1]);
  sleep(2);
  parser.launch();
  return 0;
}

NCDParser::NCDParser(char* filename):filename_(filename)
{
  ROS_INFO ("Starting NCDParser");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private ("~");

  createOdomToLeftLaserTf();
  createOdomToRightLaserTf();
  lastTime_ = -1;

  // **** parameters

  if (!nh_private.getParam ("start", start_))
    start_ = 0.0;
  if (!nh_private.getParam ("end", end_))
    end_ = -1;
  if (!nh_private.getParam ("rate", rate_))
    rate_ = 1.0;

  if (rate_ == 0) ROS_FATAL("rate parameter cannot be 0");

  // **** topics

  leftLaserPublisher_  = nh.advertise<sensor_msgs::LaserScan>("scan",  100);
  rightLaserPublisher_ = nh.advertise<sensor_msgs::LaserScan>("scan", 100);
}

NCDParser::~NCDParser()
{
  ROS_INFO ("Shutting down NCDParser");
}

void NCDParser::launch()
{
  std::ifstream aLogFile;
  aLogFile.open(filename_);

  if(!aLogFile.is_open())
    ROS_FATAL("Could not open %s\n", filename_);

  int lineCounter = 0;
  std::string line;

  // **** skip first lines

  for (int i = 0; i < 210; i++)
  {
    getline(aLogFile, line);
    lineCounter++;
  }

  // **** iterate over rest of file

  while (getline(aLogFile, line))
  {
    lineCounter++;
    std::vector<std::string> tokens;
    tokenize(line, tokens, " ");

    // skip incomplete line
    if (tokens.size() < 4) continue;

    // skip log entries before start time
    if (strtod(tokens[0].c_str(), NULL) <= start_) continue;

    // stop if time is bigger than end point time
    if (strtod(tokens[0].c_str(), NULL) > end_ && end_ != -1)
    {
      std::cout << strtod(tokens[0].c_str(), NULL) << ", " << end_ << std::endl;
      ROS_INFO("Reached specified end time.");
      break;
    }

    // publish messages
    if      (tokens[1].compare("LMS_LASER_2D_LEFT") == 0)
      publishLaserMessage(tokens, leftLaserFrame_, leftLaserPublisher_);  
    else if (tokens[1].compare("LMS_LASER_2D_RIGHT") == 0)
      publishLaserMessage(tokens, rightLaserFrame_, rightLaserPublisher_);
    else if (tokens[1].compare("ODOMETRY_POSE") == 0)
      publishTfMessages(tokens);
    else
      continue;

    // wait before publishing next message

    double time = extractValue(tokens[3], "time=");

    if(lastTime_ == -1) lastTime_ = time; 
    else
    {
      ros::Duration d = (ros::Time(time) - ros::Time(lastTime_)) * ( 1.0 / rate_);
      lastTime_ = time;
      if (d.toNSec() > 0) d.sleep();
    }
  }
}

void NCDParser::publishLaserMessage(const std::vector<std::string>& tokens,
                                    const std::string& laserFrame,
                                    const ros::Publisher& publisher)
{
  ROS_DEBUG("Laser message");

  sensor_msgs::LaserScan scan;

  double time = extractValue(tokens[3], "time=");

  scan.header.stamp    = ros::Time(time);
  scan.header.frame_id = laserFrame;

  scan.angle_min       = extractValue(tokens[3], "minAngle=") * DEG_TO_RAD; 
  scan.angle_max       = extractValue(tokens[3], "maxAngle=") * DEG_TO_RAD; 
  scan.angle_increment = extractValue(tokens[3], "angRes=")   * DEG_TO_RAD; 
  scan.range_min       = RANGE_MIN;
  scan.range_max       = RANGE_MAX;
  scan.ranges          = extractArray(tokens[3], "Range=[181]");
  scan.intensities     = extractArray(tokens[3], "Reflectance=[181]");

  publisher.publish(scan);
}

void NCDParser::publishTfMessages(const std::vector<std::string>& tokens)
{
  ROS_DEBUG("Tf message");

  // extract time
  double time = extractValue(tokens[3], "time=");

  // extract x, y, theta
  std::vector<float> xytheta = extractArray(tokens[3], "Pose=[3x1]");
  double x     = xytheta[0];
  double y     = xytheta[1];
  double z     = 0.0;
  double theta = xytheta[2];       

  // extract Pitch
  double pitch = extractValue(tokens[3], "Pitch=");

  // extract Roll
  double roll = extractValue(tokens[3], "Roll=");

  tf::Quaternion rotation;
  rotation.setRPY (roll, pitch, theta);
  worldToOdom_.setRotation (rotation);

  tf::Vector3 origin;
  origin.setValue (x, y, z);
  worldToOdom_.setOrigin (origin);

  tf::StampedTransform worldToOdomStamped(worldToOdom_, ros::Time(time), worldFrame_, odomFrame_);
  tfBroadcaster_.sendTransform(worldToOdomStamped);

  tf::StampedTransform odomToLeftLaserStamped(odomToLeftLaser_, ros::Time(time), odomFrame_, leftLaserFrame_);
  tfBroadcaster_.sendTransform(odomToLeftLaserStamped);

  tf::StampedTransform odomToRightLaserStamped(odomToRightLaser_, ros::Time(time), odomFrame_, rightLaserFrame_);
  tfBroadcaster_.sendTransform(odomToRightLaserStamped);
}

void NCDParser::tokenize (const std::string& str, 
                                std::vector <std::string> &tokens, 
                                std::string sentinel)
{
  std::string::size_type lastPos = str.find_first_not_of (sentinel, 0);
  std::string::size_type pos = str.find_first_of (sentinel, lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos)
  {
    std::string stringToken = str.substr (lastPos, pos - lastPos);
    tokens.push_back (stringToken);
    lastPos = str.find_first_not_of (sentinel, pos);
    pos = str.find_first_of (sentinel, lastPos);
  }
}

std::vector<float> NCDParser::extractArray(std::string s, std::string pattern)
{
  int n0 = s.find(pattern);
  int n1 = s.find("{", n0);
  int n2 = s.find("}", n1);
  std::string valueList = s.substr(n1+1, n2-n1-1);
  
  std::vector<float> values;
  std::vector<std::string> s_values;
  tokenize(valueList, s_values, ",");

  for (unsigned int i = 0; i < s_values.size(); i++)
    values.push_back(strtod(s_values[i].c_str(), NULL));

  return values;
}

double NCDParser::extractValue(std::string s, std::string pattern)
{
  int n1 = s.find(pattern);
  int n2 = s.find(",", n1);
  std::string s_value = s.substr(n1+pattern.length(), n2-n1-pattern.length());
  return strtod(s_value.c_str(), NULL);
}


void NCDParser::createOdomToLeftLaserTf()
{
  double x     = -0.270;
  double y     = -0.030;
  double z     =  0.495;
  double roll  =  180.0 * DEG_TO_RAD;
  double pitch =  90.0 * DEG_TO_RAD;
  double yaw   =  -90.0 * DEG_TO_RAD;

  tf::Quaternion rotation;
  rotation.setRPY (roll, pitch, yaw);
  odomToLeftLaser_.setRotation (rotation);

  tf::Vector3 origin;
  origin.setValue (x, y, z);
  odomToLeftLaser_.setOrigin (origin);
}

void NCDParser::createOdomToRightLaserTf()
{
  double x     =  0.270;
  double y     = -0.030;
  double z     =  0.495;
  double roll  =  90.0 * DEG_TO_RAD;
  double pitch =  -90.0 * DEG_TO_RAD;
  double yaw   =  180.0 * DEG_TO_RAD;

  tf::Quaternion rotation;
  rotation.setRPY (roll, pitch, yaw);
  odomToRightLaser_.setRotation (rotation);

  tf::Vector3 origin;
  origin.setValue (x, y, z);
  odomToRightLaser_.setOrigin (origin);
}
