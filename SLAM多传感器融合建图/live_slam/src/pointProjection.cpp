#include "pointProjection.h"

PointProjection::PointProjection() : firstAltitude(true) {

    subPointCloudLeft  = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudLeftTopic, 5, &PointProjection::pointCloudLeftHandler, this, ros::TransportHints().tcpNoDelay());
    subPointCloudRight = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudRightTopic, 5, &PointProjection::pointCloudRightHandler, this, ros::TransportHints().tcpNoDelay());
    subImu             = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &PointProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
    subOdom            = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &PointProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
    subAltimeter       = nh.subscribe<irp_sen_msgs::altimeter>(altimeterTopic, 200, &PointProjection::altimeterHandler, this, ros::TransportHints().tcpNoDelay());

    pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("live_slam/deskew/cloud_deskewed", 1);
    pubLaneCloud      = nh.advertise<sensor_msgs::PointCloud2>("live_slam/deskew/cloud_lane", 1);
    pubLaserCloudInfo = nh.advertise<live_slam::cloud_info> ("live_slam/deskew/cloud_info", 1);

    imu2RightLidar = rightLidar2Imu.inverse();
    imu2LeftLidar = leftLidar2Imu.inverse();

    allocateMemory();
    resetParameters();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

PointProjection::~PointProjection() {}

void PointProjection::allocateMemory() {

    pointCloudLeftIn.reset(new pcl::PointCloud<PointXYZIR>());
    pointCloudRightIn.reset(new pcl::PointCloud<PointXYZIR>());
    pointCloudLeft.reset(new pcl::PointCloud<PointXYZIR>());
    pointCloudRight.reset(new pcl::PointCloud<PointXYZIR>());
    pointCloudFull.reset(new pcl::PointCloud<PointXYZIR>());

    pointCloudPlane.reset(new pcl::PointCloud<PointXYZIR>());
    pointCloudLane.reset(new pcl::PointCloud<PointXYZIR>());

    fullCloud.reset(new pcl::PointCloud<PointType>());
    extractedCloud.reset(new pcl::PointCloud<PointType>());
    fullCloud->points.resize(N_SCAN * Horizon_SCAN);

    cloudInfo.startRingIndex.assign(N_SCAN, 0);
    cloudInfo.endRingIndex.assign(N_SCAN, 0);

    cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
    cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);

    resetParameters();
}

void PointProjection::resetParameters() {

    pointCloudLeftIn->clear();
    pointCloudRightIn->clear();
    pointCloudLeft->clear();
    pointCloudRight->clear();
    pointCloudFull->clear();
    pointCloudPlane->clear();
    pointCloudLane->clear();
    extractedCloud->clear();
    laneInd.clear();

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
}

void PointProjection::pointCloudLeftHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudLeftMsg) {

    cachePointCloudLeftQueue.push_back(*pointCloudLeftMsg);

    if (cachePointCloudLeftQueue.size() < 4)
        return;

    currentPointCloudLeftMsg = std::move(cachePointCloudLeftQueue.front());
    cachePointCloudLeftQueue.pop_front();
    pcl::moveFromROSMsg(currentPointCloudLeftMsg, *pointCloudLeftIn);

    if (pointCloudLeftIn->is_dense == false) {

        ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        ros::shutdown();
    }

    cloudHeader = currentPointCloudLeftMsg.header;
    timeScanCur = cloudHeader.stamp.toSec();

    static int ringFlagLeft = 0;

    if (ringFlagLeft == 0) {

        ringFlagLeft = -1;

        for (int i = 0; i < (int)currentPointCloudLeftMsg.fields.size(); ++i) {

            if (currentPointCloudLeftMsg.fields[i].name == "ring") {

                ringFlagLeft = 1;
                break;
            }
        }

        if (ringFlagLeft == -1) {

            ROS_ERROR("Left point cloud ring channel not available, please configure your point cloud data!");
            ros::shutdown();
        }
    }

    pcl::PointCloud<PointXYZIR>::Ptr pointCloudOut (new pcl::PointCloud<PointXYZIR>());
    pointCloudOut = transformPointCloud(pointCloudLeftIn, leftLidar2Imu);
    pointCloudLeftQueue.push_back(pointCloudOut);

    mergePointCloud();

    if (int(pointCloudFull->size()) < 20000)
        return;

    intensityCalibration();

    if (!initPose())
        return;

    projectPointCloud();

    cloudExtraction();

    publishClouds();

    resetParameters();
}

void PointProjection::pointCloudRightHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudRightMsg) {

    std::lock_guard<std::mutex> lock1(velodyneLock);

    cachePointCloudRightQueue.push_back(*pointCloudRightMsg);

    if (cachePointCloudRightQueue.size() < 5)
        return;

    currentPointCloudRightMsg = std::move(cachePointCloudRightQueue.front());
    cachePointCloudRightQueue.pop_front();
    pcl::moveFromROSMsg(currentPointCloudRightMsg, *pointCloudRightIn);

    if (pointCloudRightIn->is_dense == false) {

        ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        ros::shutdown();
    }

    static int ringFlagRight = 0;

    if (ringFlagRight == 0) {

        ringFlagRight = -1;

        for (int i = 0; i < (int)currentPointCloudRightMsg.fields.size(); ++i) {

            if (currentPointCloudRightMsg.fields[i].name == "ring") {

                ringFlagRight = 1;
                break;
            }
        }

        if (ringFlagRight == -1) {

            ROS_ERROR("Right point cloud ring channel not available, please configure your point cloud data!");
            ros::shutdown();
        }
    }

    pcl::PointCloud<PointXYZIR>::Ptr pointCloudOut (new pcl::PointCloud<PointXYZIR>());
    pointCloudOut = transformPointCloud(pointCloudRightIn, rightLidar2Imu);
    pointCloudRightQueue.push_back(pointCloudOut);

    return;

}

pcl::PointCloud<PointXYZIR>::Ptr PointProjection::transformPointCloud(pcl::PointCloud<PointXYZIR>::Ptr cloudIn, Eigen::Affine3d &transform) {

    pcl::PointCloud<PointXYZIR>::Ptr cloudOut(new pcl::PointCloud<PointXYZIR>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transform(0,0) * pointFrom.x + transform(0,1) * pointFrom.y + transform(0,2) * pointFrom.z + transform(0,3);
        cloudOut->points[i].y = transform(1,0) * pointFrom.x + transform(1,1) * pointFrom.y + transform(1,2) * pointFrom.z + transform(1,3);
        cloudOut->points[i].z = transform(2,0) * pointFrom.x + transform(2,1) * pointFrom.y + transform(2,2) * pointFrom.z + transform(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
        cloudOut->points[i].ring = pointFrom.ring;
    }
    return cloudOut;
}

//void PointProjection::intensityCalibration() {
//
//    for (int i = 0; i < (int)pointCloudFull->size(); ++i) {
//
//        if ( limitPlaneMinZ < pointCloudFull->points[i].z && pointCloudFull->points[i].z < limitPlaneMaxZ && limitPlaneMinY < pointCloudFull->points[i].y && pointCloudFull->points[i].y < limitPlaneMaxY)
//            pointCloudPlane->points.push_back(pointCloudFull->points[i]);
//
//        if ( limitLaneMinZ < pointCloudFull->points[i].z && pointCloudFull->points[i].z < limitLaneMaxZ && limitLaneMinY < pointCloudFull->points[i].y && pointCloudFull->points[i].y < limitLaneMaxY) {
//            laneInd.push_back(i);
//            pointCloudLane->points.push_back(pointCloudFull->points[i]);
//        }
//    }
//
//    modelPlane.reset(new pcl::SampleConsensusModelPlane<PointXYZIR>(pointCloudPlane));
//    ransac.reset(new pcl::RandomSampleConsensus<PointXYZIR>(modelPlane));
//    ransac->setDistanceThreshold(0.05);
//    ransac->computeModel();
//    ransac->getModelCoefficients(coefficients);
//    //cout<<"coefficients: "<<coefficients[0]<<" "<<coefficients[1]<<" "<<coefficients[2]<<" "<<coefficients[3]<<endl;
//
//    ps = sqrt(coefficients[0] * coefficients[0] + coefficients[1] * coefficients[1] + coefficients[2] * coefficients[2]);
//    pa = coefficients[0] / ps;
//    pb = coefficients[1] / ps;
//    pc = coefficients[2] / ps;
//    pd = coefficients[3] / ps;
//    //cout << "ps: " << ps << endl;
//
//    pal = imu2LeftLidar(0, 0) * pa + imu2LeftLidar(0, 1) * pb + imu2LeftLidar(0, 2) * pc;
//    pbl = imu2LeftLidar(1, 0) * pa + imu2LeftLidar(1, 1) * pb + imu2LeftLidar(1, 2) * pc;
//    pcl = imu2LeftLidar(2, 0) * pa + imu2LeftLidar(2, 1) * pb + imu2LeftLidar(2, 2) * pc;
//
//    par = imu2RightLidar(0, 0) * pa + imu2RightLidar(0, 1) * pb + imu2RightLidar(0, 2) * pc;
//    pbr = imu2RightLidar(1, 0) * pa + imu2RightLidar(1, 1) * pb + imu2RightLidar(1, 2) * pc;
//    pcr = imu2RightLidar(2, 0) * pa + imu2RightLidar(2, 1) * pb + imu2RightLidar(2, 2) * pc;
//
//    PointType thisPoint;
//    float range;
//    float intensity;
//
//    for (int j = 0; j < (int)pointCloudLane->size(); ++j) {
//
//        if(fabs(pa * pointCloudLane->points[j].x +
//                pb * pointCloudLane->points[j].y +
//                pc * pointCloudLane->points[j].z + pd) < 0.15) {
//
//            if (pointCloudLane->points[j].intensity < 15.0)
//                pointCloudFull->points[laneInd[j]].intensity = 2.5;
//
//            else {
//
//                thisPoint.intensity = pointCloudLane->points[j].intensity;
//
//                if (pointCloudLane->points[j].ring < 16) {
//
//                    thisPoint.x = imu2LeftLidar(0, 0) * pointCloudLane->points[j].x + imu2LeftLidar(0, 1) * pointCloudLane->points[j].y + imu2LeftLidar(0, 2) * pointCloudLane->points[j].z;
//                    thisPoint.y = imu2LeftLidar(1, 0) * pointCloudLane->points[j].x + imu2LeftLidar(1, 1) * pointCloudLane->points[j].y + imu2LeftLidar(1, 2) * pointCloudLane->points[j].z;
//                    thisPoint.z = imu2LeftLidar(2, 0) * pointCloudLane->points[j].x + imu2LeftLidar(2, 1) * pointCloudLane->points[j].y + imu2LeftLidar(2, 2) * pointCloudLane->points[j].z;
//                }
//
//                else {
//
//                    thisPoint.x = imu2RightLidar(0, 0) * pointCloudLane->points[j].x + imu2RightLidar(0, 1) * pointCloudLane->points[j].y + imu2RightLidar(0, 2) * pointCloudLane->points[j].z;
//                    thisPoint.y = imu2RightLidar(1, 0) * pointCloudLane->points[j].x + imu2RightLidar(1, 1) * pointCloudLane->points[j].y + imu2RightLidar(1, 2) * pointCloudLane->points[j].z;
//                    thisPoint.z = imu2RightLidar(2, 0) * pointCloudLane->points[j].x + imu2RightLidar(2, 1) * pointCloudLane->points[j].y + imu2RightLidar(2, 2) * pointCloudLane->points[j].z;
//                }
//
//                range = pointDistance(thisPoint);
//                intensity = thisPoint.intensity * pow(range, 3) / fabs(thisPoint.x * pa + thisPoint.y * pb + thisPoint.z * pc);
//                //pointCloudFull->points[laneInd[j]].intensity = min(intensity, 40.0f);
//                pointCloudFull->points[laneInd[j]].intensity = intensity / 100.0;
//                //cout << "rate: " << intensity / 60.0 << endl;
//            }
//        }
//    }
//}

void PointProjection::intensityCalibration() {

    for (int i = 0; i < (int)pointCloudFull->size(); ++i) {

        if ( limitPlaneMinZ < pointCloudFull->points[i].z && pointCloudFull->points[i].z < limitPlaneMaxZ && limitPlaneMinY < pointCloudFull->points[i].y && pointCloudFull->points[i].y < limitPlaneMaxY)
            pointCloudPlane->points.push_back(pointCloudFull->points[i]);

        if ( limitLaneMinZ < pointCloudFull->points[i].z && pointCloudFull->points[i].z < limitLaneMaxZ && limitLaneMinY < pointCloudFull->points[i].y && pointCloudFull->points[i].y < limitLaneMaxY) {
            laneInd.push_back(i);
            pointCloudLane->points.push_back(pointCloudFull->points[i]);
        }
    }

    modelPlane.reset(new pcl::SampleConsensusModelPlane<PointXYZIR>(pointCloudPlane));
    ransac.reset(new pcl::RandomSampleConsensus<PointXYZIR>(modelPlane));
    ransac->setDistanceThreshold(0.05);
    ransac->computeModel();
    ransac->getModelCoefficients(coefficients);
    //cout<<"coefficients: "<<coefficients[0]<<" "<<coefficients[1]<<" "<<coefficients[2]<<" "<<coefficients[3]<<endl;

    ps = sqrt(coefficients[0] * coefficients[0] + coefficients[1] * coefficients[1] + coefficients[2] * coefficients[2]);
    pa = coefficients[0] / ps;
    pb = coefficients[1] / ps;
    pc = coefficients[2] / ps;
    pd = coefficients[3] / ps;

    #pragma omp parallel for num_threads(numberOfCores)
    for (int j = 0; j < (int)pointCloudLane->size(); ++j) {

        if(fabs(pa * pointCloudLane->points[j].x +
                pb * pointCloudLane->points[j].y +
                pc * pointCloudLane->points[j].z + pd) < 0.15) {

            if (pointCloudLane->points[j].intensity < midIntensity)
                pointCloudFull->points[laneInd[j]].intensity = 1.0;

            else
                pointCloudFull->points[laneInd[j]].intensity = 60.0;
        }
    }
}


void PointProjection::mergePointCloud() {

    std::lock_guard<std::mutex> lock1(velodyneLock);

    if (3 < pointCloudLeftQueue.size() &&  3 < pointCloudRightQueue.size()) {

        pointCloudLeft = std::move(pointCloudLeftQueue.front());
        pointCloudLeftQueue.pop_front();
        pointCloudRight = std::move(pointCloudRightQueue.front());
        pointCloudRightQueue.pop_front();
        *pointCloudFull = *pointCloudLeft + *pointCloudRight;

    }

    else {

        ROS_DEBUG("Waiting for point cloud data ...");
        return;
    }
}

bool PointProjection::initPose() {

    if (imuQueue.empty() || timeScanCur < imuQueue.front().header.stamp.toSec()) {

        ROS_DEBUG("Waiting for IMU data ...");
        return false;
    }

    initImu();

    initOdom();

    initAltitude();

    return true;
}

void PointProjection::initImu() {

    std::lock_guard<std::mutex> lock2(imuLock);

    cloudInfo.imuAvailable = false;

    if (imuQueue.empty() || timeScanCur < imuQueue.front().header.stamp.toSec())
        return;


    while (!imuQueue.empty()) {

        if (imuQueue.front().header.stamp.toSec() <= timeScanCur)
            imuQueue.pop_front();

        else
            break;
    }

    sensor_msgs::Imu thisImuMsg = std::move(imuQueue.front());
    imuQueue.pop_front();

    double currentImuTime = thisImuMsg.header.stamp.toSec();

    if (currentImuTime <= timeScanCur)
        imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

    cloudInfo.imuAvailable = true;
}

void PointProjection::initOdom() {

    std::lock_guard<std::mutex> lock3(odomLock);

    cloudInfo.odomAvailable = false;

    if (odomQueue.empty() || timeScanCur < odomQueue.front().header.stamp.toSec())
        return;

    while (!odomQueue.empty()) {

        if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
            odomQueue.pop_front();

        else
            break;
    }

    nav_msgs::Odometry thisOdomMsg = std::move(odomQueue.front());
    odomQueue.pop_front();

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisOdomMsg.pose.pose.orientation, orientation);

    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    cloudInfo.initialGuessX = thisOdomMsg.pose.pose.position.x;
    cloudInfo.initialGuessY = thisOdomMsg.pose.pose.position.y;
    cloudInfo.initialGuessZ = thisOdomMsg.pose.pose.position.z;
    cloudInfo.initialGuessRoll  = roll;
    cloudInfo.initialGuessPitch = pitch;
    cloudInfo.initialGuessYaw   = yaw;

    cloudInfo.odomAvailable = true;
}

void PointProjection::initAltitude() {

    std::lock_guard<std::mutex> lock4(altitudeLock);

    cloudInfo.altitudeAvailable = false;
    irp_sen_msgs::altimeter thisAltimeter = std::move(altimeterQueue.front());
    altimeterQueue.pop_front();

    if (firstAltitude == true) {

        altitude = thisAltimeter.data;
        firstAltitude = false;
        // ROS_INFO("\033[1;32m----> altitude:%f.\033[0m", altitude);
    }

    cloudInfo.altitude = thisAltimeter.data - altitude;
    cloudInfo.altitudeAvailable = true;
}

void PointProjection::projectPointCloud() {

    int cloudSize = pointCloudFull->size();

    for (int i = 0; i < cloudSize; ++i) {

        PointType thisPoint;
        thisPoint.x = pointCloudFull->points[i].x;
        thisPoint.y = pointCloudFull->points[i].y;
        thisPoint.z = pointCloudFull->points[i].z;
        thisPoint.intensity = pointCloudFull->points[i].intensity;

        float range = pointDistance(thisPoint);

        if (range < lidarMinRange || lidarMaxRange < range)
            continue;

        int rowIdn = pointCloudFull->points[i].ring;

        if (rowIdn < 0 || N_SCAN <= rowIdn)
            continue;

        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        static float ang_res_x = 360.0 / float(Horizon_SCAN);
        int columnIdn = -round((horizonAngle-0.0) / ang_res_x) + Horizon_SCAN/2;

        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
            continue;

        rangeMat.at<float>(rowIdn, columnIdn) = range;

        int index = columnIdn + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;
    }
}

void PointProjection::cloudExtraction() {

    int count = 0;

    for (int i = 0; i < N_SCAN; ++i) {

        cloudInfo.startRingIndex[i] = count - 1 + 5;

        for (int j = 0; j < Horizon_SCAN; ++j) {

            if (rangeMat.at<float>(i,j) != FLT_MAX) {

                cloudInfo.pointColInd[count] = j;
                cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                ++count;
            }
        }
        cloudInfo.endRingIndex[i] = count -1 - 5;
    }
}

void PointProjection::publishClouds() {

        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
}

void PointProjection::imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg) {

    std::lock_guard<std::mutex> lock2(imuLock);
    sensor_msgs::Imu thisImu = *imuMsg;
    imuQueue.push_back(thisImu);
}

void PointProjection::odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg) {

    std::lock_guard<std::mutex> lock3(odomLock);
    odomQueue.push_back(*odometryMsg);
}

void PointProjection::altimeterHandler(const irp_sen_msgs::altimeter::ConstPtr& altimeterMsg){

    std::lock_guard<std::mutex> lock4(altitudeLock);
    altimeterQueue.push_back(*altimeterMsg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "live_slam");
    PointProjection PP;
    ROS_INFO("\033[1;32m----> Point Projection Started.\033[0m");
    ros::MultiThreadedSpinner spinner(5);
    spinner.spin();
    return 0;
}