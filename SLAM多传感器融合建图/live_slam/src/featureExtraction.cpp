#include "featureExtraction.h"

FeatureExtraction::FeatureExtraction() {

    subLaserCloudInfo = nh.subscribe<live_slam::cloud_info>("live_slam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

    pubLaserCloudInfo = nh.advertise<live_slam::cloud_info> ("live_slam/feature/cloud_info", 1);
    pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("live_slam/feature/cloud_corner", 1);
    pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("live_slam/feature/cloud_surface", 1);
    pubLaneCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("live_slam/feature/cloud_lane_corner", 1);
    pubLaneSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("live_slam/feature/cloud_lane_surface", 1);

    initializationValue();
}

FeatureExtraction::~FeatureExtraction() {}

void FeatureExtraction::initializationValue() {

    cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

    downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

    extractedCloud.reset(new pcl::PointCloud<PointType>());
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());
    laneCornerCloud.reset(new pcl::PointCloud<PointType>());
    laneSurfaceCloud.reset(new pcl::PointCloud<PointType>());


    cloudCurvature = new float[N_SCAN * Horizon_SCAN];
    cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
    cloudLabel = new int[N_SCAN * Horizon_SCAN];
    cloudLaneMarkingLabel = new int[N_SCAN * Horizon_SCAN];
}

void FeatureExtraction::laserCloudInfoHandler(const live_slam::cloud_infoConstPtr& msgIn) {

    cloudInfo = *msgIn; // new cloud info
    cloudHeader = msgIn->header; // new cloud header
    pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction

    calculateSmoothness();

    markOccludedPoints();

    extractFeatures();

    publishFeatureCloud();
}

void FeatureExtraction::calculateSmoothness() {

    int cloudSize = extractedCloud->points.size();
    float diffRange;
    float diffIntensity;
    float intensity1;
    float intensity2;
    float intensity3;

    laneCornerCloud->clear();
    laneSurfaceCloud->clear();

    for (int i = 5; i < cloudSize - 5; i++) {

        diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                  + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                  + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                  + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                  + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                  + cloudInfo.pointRange[i+5];

        cloudCurvature[i] = diffRange * diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
        cloudLaneMarkingLabel[i] = -1;
        // cloudSmoothness for sorting
        cloudSmoothness[i].value = cloudCurvature[i];
        cloudSmoothness[i].ind = i;

        intensity1 = extractedCloud->points[i].intensity;
        intensity2 = extractedCloud->points[i-1].intensity;
        intensity3 = extractedCloud->points[i+1].intensity;


        if (fabs( intensity1 - 60.0) < 0.1) {

            diffIntensity = fabs(intensity1 * 2 - intensity2 - intensity3);

            if (fabs(diffIntensity - 59.0) < 0.1)
                laneCornerCloud->push_back(extractedCloud->points[i]);

            else if (fabs(diffIntensity) < 0.1)
                laneSurfaceCloud->push_back(extractedCloud->points[i]);
        }
    }
}

void FeatureExtraction::markOccludedPoints() {

    int cloudSize = extractedCloud->points.size();
    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i) {

        // occluded points
        float depth1 = cloudInfo.pointRange[i];
        float depth2 = cloudInfo.pointRange[i+1];
        int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));

        if (columnDiff < 10){
            // 10 pixel diff in range image
            if (depth1 - depth2 > 0.3) {

                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }

            else if (depth2 - depth1 > 0.3) {

                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        }
        // parallel beam
        float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
        float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

        if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
            cloudNeighborPicked[i] = 1;


    }
}

void FeatureExtraction::extractFeatures() {

    cornerCloud->clear();
    surfaceCloud->clear();

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    for (int i = 0; i < N_SCAN; i++) {

        surfaceCloudScan->clear();

        for (int j = 0; j < 6; j++) {

            int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
            int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;

            std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

            int largestPickedNum = 0;

            for (int k = ep; k >= sp; k--) {

                int ind = cloudSmoothness[k].ind;

                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold) {

                    largestPickedNum++;

                    if (largestPickedNum <= 20){

                        cloudLabel[ind] = 1;
                        cornerCloud->push_back(extractedCloud->points[ind]);
                    }

                    else break;

                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++) {

                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));

                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }

                    for (int l = -1; l >= -5; l--) {

                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));

                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {

                int ind = cloudSmoothness[k].ind;

                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold) {

                    cloudLabel[ind] = -1;
                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++) {

                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }

                    for (int l = -1; l >= -5; l--) {

                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {

                if (cloudLabel[k] <= 0)
                    surfaceCloudScan->push_back(extractedCloud->points[k]);
            }


        }

        surfaceCloudScanDS->clear();
        downSizeFilter.setInputCloud(surfaceCloudScan);
        downSizeFilter.filter(*surfaceCloudScanDS);

        *surfaceCloud += *surfaceCloudScanDS;
    }
}

void FeatureExtraction::freeCloudInfoMemory() {

    cloudInfo.startRingIndex.clear();
    cloudInfo.endRingIndex.clear();
    cloudInfo.pointColInd.clear();
    cloudInfo.pointRange.clear();
}

void FeatureExtraction::publishFeatureCloud() {

    // free cloud info memory
    freeCloudInfoMemory();
    // save newly extracted features
    cloudInfo.cloud_corner  = publishCloud(&pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
    cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
    cloudInfo.cloud_lane_corner = publishCloud(&pubLaneCornerPoints, laneCornerCloud, cloudHeader.stamp, lidarFrame);
    cloudInfo.cloud_lane_surface = publishCloud(&pubLaneSurfacePoints, laneSurfaceCloud, cloudHeader.stamp, lidarFrame);


    // publish to mapOptimization
    pubLaserCloudInfo.publish(cloudInfo);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "live_slam");

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");

    ros::spin();

    return 0;
}