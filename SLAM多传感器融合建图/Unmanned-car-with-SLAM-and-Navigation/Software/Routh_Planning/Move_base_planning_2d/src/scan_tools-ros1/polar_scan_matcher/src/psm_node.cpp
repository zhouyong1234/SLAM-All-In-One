/*
*  Polar Scan Matcher
*  Copyright (C) 2010, CCNY Robotics Lab
*  Ivan Dryanovski <ivan.dryanovski@gmail.com>
*  William Morris <morris@ee.ccny.cuny.edu>
*  http://robotics.ccny.cuny.edu
*  Modified 2014, Daniel Axtens <daniel@axtens.net>
*  whilst a student at the Australian National University
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  This is a wrapper around Polar Scan Matcher [1], written by 
*  Albert Diosi
*
*  [1] A. Diosi and L. Kleeman, "Laser Scan Matching in Polar Coordinates with 
*  Application to SLAM " Proceedings of 2005 IEEE/RSJ International Conference 
*  on Intelligent Robots and Systems, August, 2005, Edmonton, Canada
*/

#include "polar_scan_matcher/psm_node.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "PolarScanMatching Node");
  PSMNode psmNode;
  ros::spin();
}

PSMNode::PSMNode()
{
  ROS_INFO("Creating PolarScanMatching node");

  ros::NodeHandle nh;

  initialized_   = false;
  totalDuration_ = 0.0;
  scansCount_    = 0;

  prevWorldToBase_.setIdentity();

  getParams();  

  scanSubscriber_ = nh.subscribe (scanTopic_, 10, &PSMNode::scanCallback, this);
  imuSubscriber_  = nh.subscribe (imuTopic_,  10, &PSMNode::imuCallback,  this);
  posePublisher_  = nh.advertise<geometry_msgs::Pose2D>(poseTopic_, 10);
}

PSMNode::~PSMNode()
{
  ROS_INFO("Destroying PolarScanMatching node");
}

void PSMNode::getParams()
{
  ros::NodeHandle nh_private("~");

  std::string odometryType;

  // **** wrapper parameters
  
  if (!nh_private.getParam ("world_frame", worldFrame_))
    worldFrame_ = "world";
  if (!nh_private.getParam ("base_frame", baseFrame_))
    baseFrame_ = "base_link";
  if (!nh_private.getParam ("publish_tf", publishTf_))
    publishTf_ = true;
  if (!nh_private.getParam ("publish_pose", publishPose_))
    publishPose_ = true;
  if (!nh_private.getParam ("odometry_type", odometryType))
    odometryType = "none";

  if (odometryType.compare("none") == 0)
  {
    useTfOdometry_  = false;
    useImuOdometry_ = false;
  }
  else if (odometryType.compare("tf") == 0)
  {
    useTfOdometry_  = true;
    useImuOdometry_ = false;
  }
  else if (odometryType.compare("imu") == 0)
  {
    useTfOdometry_  = false;
    useImuOdometry_ = true;
  }
  else
  {
    ROS_WARN("Unknown value of odometry_type parameter passed to psm_node. \
              Using default value (\"none\")");
    useTfOdometry_  = false;
    useImuOdometry_ = false;
  }

  // **** PSM parameters

  if (!nh_private.getParam ("min_valid_points", minValidPoints_))
    minValidPoints_ = 200;
  if (!nh_private.getParam ("search_window", searchWindow_))
    searchWindow_ = 40;
  if (!nh_private.getParam ("max_error", maxError_))
    maxError_ = 0.20;
  if (!nh_private.getParam ("max_iterations", maxIterations_))
    maxIterations_ = 20;
  if (!nh_private.getParam ("stop_condition", stopCondition_))
    stopCondition_ = 0.01;
}

bool PSMNode::initialize(const sensor_msgs::LaserScan& scan)
{
  laserFrame_ = scan.header.frame_id;

  // **** get base to laser tf

  tf::StampedTransform baseToLaserTf;
  try
  {
   tfListener_.waitForTransform(baseFrame_, scan.header.frame_id, scan.header.stamp, ros::Duration(1.0));
   tfListener_.lookupTransform (baseFrame_, scan.header.frame_id, scan.header.stamp, baseToLaserTf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("ScanMatcherNode: Could get initial laser transform, skipping scan (%s)", ex.what());
    return false;
  }
  baseToLaser_ = baseToLaserTf;
  laserToBase_ = baseToLaser_.inverse();

  // **** pass parameters to matcher and initialise

  matcher_.PM_L_POINTS         = scan.ranges.size();

  matcher_.PM_FOV              = (scan.angle_max - scan.angle_min) * 180.0 / M_PI;
  matcher_.PM_MAX_RANGE        = scan.range_max * ROS_TO_PM;

  matcher_.PM_TIME_DELAY       = 0.00;

  matcher_.PM_MIN_VALID_POINTS = minValidPoints_;
  matcher_.PM_SEARCH_WINDOW    = searchWindow_;
  matcher_.PM_MAX_ERROR        = maxError_ * ROS_TO_PM;

  matcher_.PM_MAX_ITER         = maxIterations_;
  matcher_.PM_MAX_ITER_ICP     = maxIterations_;
  matcher_.PM_STOP_COND        = stopCondition_ * ROS_TO_PM;
  matcher_.PM_STOP_COND_ICP    = stopCondition_ * ROS_TO_PM;

  matcher_.pm_init();

  // **** get the initial worldToBase tf

  getCurrentEstimatedPose(prevWorldToBase_, scan);

  // **** create the first pm scan from the laser scan message

  tf::Transform t;
  t.setIdentity();
  prevPMScan_ = new PMScan(scan.ranges.size());
  rosToPMScan(scan, t, prevPMScan_);

  return true;
}

void PSMNode::imuCallback (const sensor_msgs::Imu& imuMsg)
{
  imuMutex_.lock();
  tf::Quaternion q(imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w);
  tf::Matrix3x3 m(q);
  double temp;
  m.getRPY(temp, temp, currImuAngle_);
  imuMutex_.unlock();
}

void PSMNode::scanCallback(const sensor_msgs::LaserScan& scan)
{
  ROS_DEBUG("Received scan");
  scansCount_++;

  struct timeval start, end;
  gettimeofday(&start, NULL);

  // **** if this is the first scan, initialize and leave the function here

  if (!initialized_)
  {   
    initialized_ = initialize(scan);
    if (initialized_) ROS_INFO("Matcher initialized");
    return;
  }
  
  // **** attmempt to match the two scans

  // PM scan matcher is used in the following way:
  // The reference scan (prevPMScan_) always has a pose of 0
  // The new scan (currPMScan) has a pose equal to the movement
  // of the laser in the world frame since the last scan (tf::Transform change)
  // The computed correction is then propagated using the tf machinery

  prevPMScan_->rx = 0;
  prevPMScan_->ry = 0;
  prevPMScan_->th = 0; 

  tf::Transform currWorldToBase;
  tf::Transform change;
  change.setIdentity();

  // what odometry model to use
  if (useTfOdometry_) 
  {
    // get the current position of the base in the world frame
    // if no transofrm is available, we'll use the last known transform

    getCurrentEstimatedPose(currWorldToBase, scan);
    change = laserToBase_ * prevWorldToBase_.inverse() * currWorldToBase * baseToLaser_;
  }
  else if (useImuOdometry_)
  {
    imuMutex_.lock();
    double dTheta = currImuAngle_ - prevImuAngle_;
    prevImuAngle_ = currImuAngle_;
    change.getRotation().setRPY(0.0, 0.0, dTheta);
    imuMutex_.unlock();
  }

  PMScan * currPMScan = new PMScan(scan.ranges.size());
  rosToPMScan(scan, change, currPMScan);
  
  try
  {         
    matcher_.pm_psm(prevPMScan_, currPMScan);                         
  }
  catch(int err)
  {
    ROS_WARN("Error in scan matching");
    delete prevPMScan_;
    prevPMScan_ = currPMScan;
    return;
  };    

  // **** calculate change in position

  // rotate by -90 degrees, since polar scan matcher assumes different laser frame
  // and scale down by 100
  double dx =  currPMScan->ry / ROS_TO_PM;
  double dy = -currPMScan->rx / ROS_TO_PM;
  double da =  currPMScan->th; 

  // change = scan match result for how much laser moved between scans, 
  // in the world frame
  change.setOrigin(tf::Vector3(dx, dy, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, da);
  change.setRotation(q);
  
  // **** publish the new estimated pose as a tf
   
  currWorldToBase = prevWorldToBase_ * baseToLaser_ * change * laserToBase_;

  if (publishTf_  ) publishTf  (currWorldToBase, scan.header.stamp);
  if (publishPose_) publishPose(currWorldToBase);

  // **** swap old and new

  delete prevPMScan_;
  prevPMScan_      = currPMScan;
  prevWorldToBase_ = currWorldToBase;

  // **** timing information - needed for profiling only

  gettimeofday(&end, NULL);
  double dur = ((end.tv_sec   * 1000000 + end.tv_usec  ) - 
                (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
  totalDuration_ += dur;
  double ave = totalDuration_/scansCount_;

  ROS_INFO("dur:\t %.3f ms \t ave:\t %.3f ms", dur, ave);
}

void PSMNode::publishTf(const tf::Transform& transform, 
                                  const ros::Time& time)
{
  tf::StampedTransform transformMsg (transform, time, worldFrame_, baseFrame_);
  tfBroadcaster_.sendTransform (transformMsg);
}

void PSMNode::publishPose(const tf::Transform& transform)
{
  geometry_msgs::Pose2D pose;
  tfToPose2D(transform, pose);

  posePublisher_.publish(pose);
}

void PSMNode::rosToPMScan(const sensor_msgs::LaserScan& scan, 
                          const tf::Transform& change,
                                PMScan* pmScan)
{
  geometry_msgs::Pose2D pose;
  tfToPose2D(change, pose);

  // FIXME: rotate x & y by 90 degree?

  pmScan->rx = pose.x * ROS_TO_PM;
  pmScan->ry = pose.y * ROS_TO_PM;
  pmScan->th = pose.theta;

  for (int i = 0; i < scan.ranges.size(); ++i)
  {
    if (scan.ranges[i] == 0) 
    {
      pmScan->r[i] = 99999;  // hokuyo uses 0 for out of range reading
    }
    else
    {
      pmScan->r[i] = scan.ranges[i] * ROS_TO_PM;
      pmScan->x[i] = (pmScan->r[i]) * matcher_.pm_co[i];
      pmScan->y[i] = (pmScan->r[i]) * matcher_.pm_si[i];
      pmScan->bad[i] = 0;
    }

    pmScan->bad[i] = 0;
  }

  matcher_.pm_median_filter  (pmScan);
  matcher_.pm_find_far_points(pmScan);
  matcher_.pm_segment_scan   (pmScan);  
}

void PSMNode::getCurrentEstimatedPose(tf::Transform& worldToBase, 
                                      const sensor_msgs::LaserScan& scanMsg)
{
  tf::StampedTransform worldToBaseTf;
  try
  {
     tfListener_.lookupTransform (worldFrame_, baseFrame_, scanMsg.header.stamp, worldToBaseTf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - use the pose from our last estimation
    ROS_WARN("Transform unavailable, using last estimated pose (%s)", ex.what());
    worldToBase = prevWorldToBase_;
    return;
  }
  worldToBase = worldToBaseTf;
}

void PSMNode::pose2DToTf(const geometry_msgs::Pose2D& pose, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(pose.x, pose.y, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, pose.theta);
  t.setRotation(q);
}

void PSMNode::tfToPose2D(const tf::Transform& t, geometry_msgs::Pose2D& pose)
{
  tf::Matrix3x3 m(t.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  pose.x = t.getOrigin().getX();
  pose.y = t.getOrigin().getY();
  pose.theta = yaw;
}
