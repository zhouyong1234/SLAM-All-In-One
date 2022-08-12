#ifndef LIVE_SLAM_WS_IMUPREINTEGRATION_H
#define LIVE_SLAM_WS_IMUPREINTEGRATION_H

#include "paramServer.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)



class TransformFusion : public ParamServer {
public:
    std::mutex mtx;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;

    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;

    double lidarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;

    TransformFusion();
    ~TransformFusion();
    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom);
    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
};

class IMUPreintegration : public ParamServer {
public:
    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(0, 0, 0));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(0, 0, 0));

    IMUPreintegration();
    ~IMUPreintegration();
    void resetOptimization();
    void resetParams();
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw);
};

#endif //LIVE_SLAM_WS_IMUPREINTEGRATION_H
