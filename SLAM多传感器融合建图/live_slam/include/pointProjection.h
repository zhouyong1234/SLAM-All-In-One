#ifndef LIVE_SLAM_POINTPROJECTION_H
#define LIVE_SLAM_POINTPROJECTION_H

#include "paramServer.h"

struct VelodynePointXYZIR {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (
    VelodynePointXYZIR,
   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
   (uint16_t, ring, ring)
)

using PointXYZIR = VelodynePointXYZIR;

const int queueLength = 2000;

class PointProjection : public ParamServer {

public:
    ros::Subscriber subPointCloudLeft;
    ros::Subscriber subPointCloudRight;
    ros::Subscriber subImu;
    ros::Subscriber subOdom;
    ros::Subscriber subAltimeter;

    ros::Publisher  pubExtractedCloud;
    ros::Publisher  pubLaneCloud;
    ros::Publisher  pubLaserCloudInfo;

    sensor_msgs::PointCloud2 currentPointCloudLeftMsg;
    sensor_msgs::PointCloud2 currentPointCloudRightMsg;

    pcl::PointCloud<PointXYZIR>::Ptr pointCloudLeftIn;
    pcl::PointCloud<PointXYZIR>::Ptr pointCloudRightIn;

    pcl::PointCloud<PointXYZIR>::Ptr pointCloudLeft;
    pcl::PointCloud<PointXYZIR>::Ptr pointCloudRight;
    pcl::PointCloud<PointXYZIR>::Ptr pointCloudFull;
    pcl::PointCloud<PointXYZIR>::Ptr pointCloudPlane;

    pcl::SampleConsensusModelPlane<PointXYZIR>::Ptr modelPlane;
    pcl::RandomSampleConsensus<PointXYZIR>::Ptr ransac;
    Eigen::VectorXf coefficients;

    pcl::PointCloud<PointXYZIR>::Ptr pointCloudLane;
    vector<int> laneInd;

    float pa;
    float pb;
    float pc;
    float pd;
    float ps;

    float par;
    float pbr;
    float pcr;

    float pal;
    float pbl;
    float pcl;

    Eigen::Affine3d imu2RightLidar;
    Eigen::Affine3d imu2LeftLidar;

    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;
    cv::Mat rangeMat;

    deque<sensor_msgs::PointCloud2> cachePointCloudLeftQueue;
    deque<sensor_msgs::PointCloud2> cachePointCloudRightQueue;
    deque<pcl::PointCloud<PointXYZIR>::Ptr> pointCloudLeftQueue;
    deque<pcl::PointCloud<PointXYZIR>::Ptr> pointCloudRightQueue;
    std::deque<sensor_msgs::Imu> imuQueue;
    std::deque<nav_msgs::Odometry> odomQueue;
    std::deque<irp_sen_msgs::altimeter> altimeterQueue;

    bool firstAltitude;
    float altitude;

    double timeScanCur;
    std_msgs::Header cloudHeader;
    live_slam::cloud_info cloudInfo;

    std::mutex velodyneLock;
    std::mutex imuLock;
    std::mutex odomLock;
    std::mutex altitudeLock;

    PointProjection();
    ~PointProjection();

    void allocateMemory();
    void resetParameters();
    void pointCloudLeftHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    void pointCloudRightHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudRightMsg);
    pcl::PointCloud<PointXYZIR>::Ptr transformPointCloud(pcl::PointCloud<PointXYZIR>::Ptr cloudIn, Eigen::Affine3d &transform);
    void intensityCalibration();
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg);
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg);
    void altimeterHandler(const irp_sen_msgs::altimeter::ConstPtr& altimeterMsg);
    void mergePointCloud();
    bool initPose();
    void initImu();
    void initOdom();
    void initAltitude();
    void projectPointCloud();
    void cloudExtraction();
    void publishClouds();
};

#endif //LIVE_SLAM_POINTPROJECTION_H
