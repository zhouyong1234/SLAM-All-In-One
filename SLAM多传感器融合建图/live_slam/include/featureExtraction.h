#ifndef LIVE_SLAM_WS_FEATUREEXTRACTION_H
#define LIVE_SLAM_WS_FEATUREEXTRACTION_H

#include "paramServer.h"

struct smoothness_t{
    float value;
    size_t ind;
};

struct by_value{
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer {

public:
    ros::Subscriber subLaserCloudInfo;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;
    ros::Publisher pubLaneCornerPoints;
    ros::Publisher pubLaneSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;
    pcl::PointCloud<PointType>::Ptr laneCornerCloud;
    pcl::PointCloud<PointType>::Ptr laneSurfaceCloud;


    pcl::VoxelGrid<PointType> downSizeFilter;

    live_slam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;
    int *cloudLaneMarkingLabel;

    FeatureExtraction();
    ~FeatureExtraction();

    void initializationValue();
    void laserCloudInfoHandler(const live_slam::cloud_infoConstPtr& msgIn);
    void calculateSmoothness();
    void markOccludedPoints();
    void extractFeatures();
    void freeCloudInfoMemory();
    void publishFeatureCloud();
};

#endif //LIVE_SLAM_WS_FEATUREEXTRACTION_H
