#pragma once
#ifndef LIVE_SLAM_PARAMSERVER_H
#define LIVE_SLAM_PARAMSERVER_H

#define PCL_NO_PRECOMPILE
#include "live_slam/cloud_info.h"
#include "live_slam/save_map.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <irp_sen_msgs/altimeter.h>
#include <irp_sen_msgs/vrs.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

class ParamServer {

public:
    ros::NodeHandle nh;

    string pointCloudLeftTopic;
    string pointCloudRightTopic;
    string imuTopic;
    string odomTopic;
    string encoderTopic;
    string altimeterTopic;
    string gpsTopic;

    string lidarFrame;
    string baseLinkFrame;
    string odometryFrame;
    string mapFrame;

    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    bool savePCD;
    string savePCDDirectory;

    int N_SCAN;
    int Horizon_SCAN;
    int downSampleRate;
    float lidarMinRange;
    float lidarMaxRange;
    float inclinationAngleLeft;
    float inclinationAngleRight;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    float zWeight;

    vector<double> vehicleToLeftVelodyneTransform;
    Eigen::Matrix4d vehicleToLeftVelodyne;

    vector<double> vehicleToRightVelodyneTransform;
    Eigen::Matrix4d vehicleToRightVelodyne;

    vector<double> vehicleToImuTransform;
    Eigen::Matrix4d vehicleToImu;

    vector<double> vehicleToGpsTransform;
    Eigen::Matrix4d vehicleToGps;

    vector<double> vehicleToVrsTransform;
    Eigen::Matrix4d vehicleToVrs;

    vector<double> gps2OdomTransform;
    Eigen::Matrix4d gps2Odom;

    Eigen::Affine3d rightLidar2Imu;
    Eigen::Affine3d leftLidar2Imu;
    Eigen::Affine3d gps2Imu;
    Eigen::Affine3d vrs2Imu;

    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    float limitPlaneMinZ;
    float limitPlaneMaxZ;
    float limitPlaneMinY;
    float limitPlaneMaxY;

    float limitLaneMinZ;
    float limitLaneMaxZ;
    float limitLaneMinY;
    float limitLaneMaxY;

    float midIntensity;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance;
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer(){

        nh.param<std::string>("live_slam/pointCloudLeftTopic",  pointCloudLeftTopic, "ns2/velodyne_points");
        nh.param<std::string>("live_slam/pointCloudRightTopic", pointCloudRightTopic, "ns1/velodyne_points");
        nh.param<std::string>("live_slam/imuTopic",             imuTopic, "imu/data_raw");
        nh.param<std::string>("live_slam/odomTopic",            odomTopic, "odometry/imu");
        nh.param<std::string>("live_slam/encoderTopic",         encoderTopic, "encoder_count");
        nh.param<std::string>("live_slam/altimeterTopic",         altimeterTopic, "altimeter_data");
        nh.param<std::string>("live_slam/gpsTopic",             gpsTopic, "gps/fix");

        nh.param<std::string>("live_slam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("live_slam/baseLinkFrame", baseLinkFrame, "base_link");
        nh.param<std::string>("live_slam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("live_slam/mapFrame", mapFrame, "map");

        nh.param<bool>("live_slam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("live_slam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("live_slam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("live_slam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("live_slam/savePCD", savePCD, false);
        nh.param<std::string>("live_slam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        nh.param<int>("live_slam/N_SCAN", N_SCAN, 16);
        nh.param<int>("live_slam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("live_slam/downSampleRate", downSampleRate, 1);
        nh.param<float>("live_slam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("live_slam/lidarMaxRange", lidarMaxRange, 1000.0);
        nh.param<float>("live_slam/inclinationAngleLeft", inclinationAngleLeft, 45.0);
        nh.param<float>("live_slam/inclinationAngleRight", inclinationAngleRight, 45.0);

        nh.param<float>("live_slam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("live_slam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("live_slam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("live_slam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("live_slam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("live_slam/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<float>("live_slam/zWeight", zWeight, 0.99);

        nh.param<vector<double>>("live_slam/vehicleToLeftVelodyneTransform", vehicleToLeftVelodyneTransform,vector<double>());
        vehicleToLeftVelodyne = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(vehicleToLeftVelodyneTransform.data(), 4, 4);
        nh.param<vector<double>>("live_slam/vehicleToRightVelodyneTransform", vehicleToRightVelodyneTransform,vector<double>());
        vehicleToRightVelodyne = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(vehicleToRightVelodyneTransform.data(), 4, 4);
        nh.param<vector<double>>("live_slam/vehicleToImuTransform", vehicleToImuTransform,vector<double>());
        vehicleToImu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(vehicleToImuTransform.data(), 4, 4);
        nh.param<vector<double>>("live_slam/vehicleToGpsTransform", vehicleToGpsTransform,vector<double>());
        vehicleToGps = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(vehicleToGpsTransform.data(), 4, 4);
        nh.param<vector<double>>("live_slam/vehicleToVrsTransform", vehicleToVrsTransform,vector<double>());
        vehicleToGps = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(vehicleToVrsTransform.data(), 4, 4);
        nh.param<vector<double>>("live_slam/gps2OdomTransform", gps2OdomTransform,vector<double>());
        gps2Odom = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(gps2OdomTransform.data(), 4, 4);

        nh.param<float>("live_slam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("live_slam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("live_slam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("live_slam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("live_slam/limitPlaneMinZ", limitPlaneMinZ, -10.0);
        nh.param<float>("live_slam/limitPlaneMaxZ", limitPlaneMaxZ, -1.0);

        nh.param<float>("live_slam/limitPlaneMinY", limitPlaneMinY, -1.0);
        nh.param<float>("live_slam/limitPlaneMaxY", limitPlaneMaxY, 1.0);

        nh.param<float>("live_slam/limitLaneMinZ", limitLaneMinZ, -10.0);
        nh.param<float>("live_slam/limitLaneMaxZ", limitLaneMaxZ, -1.4);

        nh.param<float>("live_slam/limitLaneMinY", limitLaneMinY, -10.0);
        nh.param<float>("live_slam/limitLaneMaxY", limitLaneMaxY, 10.0);

        nh.param<float>("live_slam/midIntensity", midIntensity, 30.0);


        nh.param<float>("live_slam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("live_slam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("live_slam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("live_slam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("live_slam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("live_slam/numberOfCores", numberOfCores, 2);
        nh.param<double>("live_slam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("live_slam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("live_slam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("live_slam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("live_slam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("live_slam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("live_slam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("live_slam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("live_slam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("live_slam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("live_slam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("live_slam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("live_slam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("live_slam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("live_slam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        rightLidar2Imu.matrix() = vehicleToImu.inverse() * vehicleToRightVelodyne;
        leftLidar2Imu.matrix()  = vehicleToImu.inverse() * vehicleToLeftVelodyne;
        gps2Imu.matrix()        = vehicleToImu.inverse() * vehicleToGps;
        vrs2Imu.matrix()        = vehicleToImu.inverse() * vehicleToVrs;


        usleep(100);
    }

    ~ParamServer(){}
};

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame) {

    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;

    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);

    return tempCloud;
}

template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw) {

    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

template<typename T>
double ROS_TIME(T msg) {

    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z) {

    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z) {

    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

float pointDistance(PointType p) {

    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2) {

    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif //LIVE_SLAM_PARAMSERVER_H
