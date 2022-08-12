#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

///
///@brief 利用 IMU 里程计估计的关键帧到最新时间的位姿变换更新全局优化估计的 Lidar 里程计位姿
///
///
class TransformFusion : public ParamServer
{
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

    TransformFusion()
    {
        // 如果 Lidar 系不是baselink 则需要 lidar 和 baselink 之间的转换
        if(lidarFrame != baselinkFrame)
        {
            try
            {
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
        }

        // 接收后端节点估计的 Lidar 里程计消息（包含全局回环优化）
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        // 接收 IMU 预积分节点估计的最新 IMU 里程计位姿估计
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        pubImuPath       = nh.advertise<nav_msgs::Path>    ("lio_sam/imu/path", 1);
    }

    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        // 更新全局优化后的里程计位姿以及对应时间
        lidarOdomAffine = odom2affine(*odomMsg);
        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // static tf
        // map 系和 odom 系重合
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);

        // 将 IMU 增量里程计消息加入缓存队列中
        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1)
            return;
        // 将当前 Lidar 关键帧时间之前的 IMU 里程计估计去掉
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }

        // 通过离当前 Lidar 关键帧时间最近的 IMU 里程计和最新的 IMU 里程计位姿，计算当前 Lidar 关键帧时刻到最新时刻的相对位姿变换
        // 以此更新全局优化后的位姿估计
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
        
        // publish latest odometry
        // 发布最新的 IMU 里程计估计（考虑全局优化）
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // publish tf
        // 发布里程计到 baselink 的 tf
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // publish IMU path
        // 更新 IMU 里程计的路径（从当前 Lidar 关键帧到最新 IMU 消息之间的路径）作为预测显示
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

///
///@brief 接收两个消息：
/// 1. IMU 消息：将消息存入缓存队列中，并利用上一个 Lidar 关键帧的最优估计和该时刻到当前时刻的预积分观测预测当前时刻的状态，并发布和 IMU 同频的位姿估计
/// 2. 从后端接收当前关键帧时刻位姿（基于增量计算）的全局估计：对上一关键帧和当前关键帧的 IMU 消息进行预积分计算，构建关键帧帧之间的相对位姿并进行优化 
///
class IMUPreintegration : public ParamServer
{
public:

    // 线程锁
    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;     // 先验位姿噪声
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;      // 先验速度噪声
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;     // 先验零偏噪声
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;    // 正常状态下的 Lidar 里程计先验噪声
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;   // 失效状态下的 Lidar 里程计先验噪声
    gtsam::Vector noiseModelBetweenBias;                        // IMU 的随机噪声模型


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;     // 用于优化的 IMU 预积分接口
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;     // 用于于估计帧间位姿变换的 IMU 预积分接口

    std::deque<sensor_msgs::Imu> imuQueOpt;              // 用于优化的 IMU 缓存队列
    std::deque<sensor_msgs::Imu> imuQueImu;              // 用于于估计帧间位姿变换的 IMU 消息队列

    // 前一时刻的位姿、速度、状态、和零偏
    gtsam::Pose3 prevPose_;                  
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    // 前一时刻里程计和状态和零偏
    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    // gtsam 优化器、因子图以及变量节点
    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    // IMU 和 Lidar 之间的外参
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    IMUPreintegration()
    {
        // 接收原始 IMU 消息
        subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic,                   2000, &IMUPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        
        // 接收后端节点估计的 Lidar 里程计消息（利用帧间增量变化计算的全局位姿，不包含回环优化）
        subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        // 获得当前里程计时间
        double currentCorrectionTime = ROS_TIME(odomMsg);

        // make sure we have imu data to integrate
        if (imuQueOpt.empty())
            return;

        // 使用里程计消息构建 gtsam 变量节点格式，通过协方差标志位判断位姿估计是否退化
        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        // 0. initialize system
        if (systemInitialized == false)
        {
            // 重置优化相关选项和变量，包括因子图以及优化变量
            resetOptimization(); 

            // pop old IMU message -- 清理当前帧时间之前的 IMU 消息
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }

            // initial pose
            // 获取当前关键帧时间 IMU 位姿作为下一次预积分优化的起点
            prevPose_ = lidarPose.compose(lidar2Imu);
            // 将当前关键帧对应 IMU 位姿作为对第一个位姿的先验估计加入因子图中，先验噪声为 1e-2 （置信度较高）的 6 维对角阵
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);

            // initial velocity
            // 速度的先验估计设为 0，先验噪声是 1e4 （由于不知道速度的信息，因此置信度较低）的 3 维对角阵
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);

            // initial bias
            // 先验零偏设为 0，先验噪声为 1e-3 的 6 维（加速度计 3 维、陀螺仪 3 维）对角阵
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);

            // add values -- 添加先验变量节点
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);

            // optimize once -- 用这些因子和变量更新 iSAM 优化器状态，理论上这一次不会改变这些信息
            optimizer.update(graphFactors, graphValues);

            // 清空当前因子和变量
            graphFactors.resize(0);
            graphValues.clear();

            // 使用当前设置的零偏重置 gtsam 预积分接口
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            systemInitialized = true;
            return;
        }


        // reset graph for speed
        // 每隔 100 帧进行一次边缘化
        if (key == 100)
        {
            // get updated noise before reset -- 将 100 个关键帧边缘化，获得边缘后的位姿、速度和零偏的协方差矩阵（噪声）
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();

            // 将上一关键帧的位姿、速度和零偏和对应的边缘后的协方差加入重置化后的因子图中
            // 用这些因子和上一关键帧的状态估计作为变量更新优化器，和系统初始化过程一样
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }


        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            // 将上一次优化时间和当前关键帧时间之间的 IMU 消息进行预积分计算，并将其从优化队列中去掉
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                // 计算前后两帧之间的时间差，（理论上 lastImuT_opt 不会小于 0）
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // add imu factor to graph
        // 构建 IMU 预积分因子，约束前后两帧的位姿、速度以及前一帧的零偏
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
    
        // add imu bias between factor
        // 构建前后两帧的零偏因子，约束两帧之间的零偏基本不变，噪声模型考虑前后两帧之间的时间差和 IMU 本身的噪声模型，时间差越大，由于随机游走的影响不确定度越高
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

        // add pose factor
        // 将当前帧的后端位姿估计作为当前帧位姿的先验估计加入因子图中，噪声模型根据估计是否退化进行选择
        // 如果退化：噪声为 1 的 6 维对角阵
        // 如果没有退化：姿态噪声设为 0.05 rad，位置噪声设为 0.1m
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);

        // insert predicted values
        // 利用 IMU 预积分观测从上一阵的状态和零偏估计当前帧的状态（位姿、速度、零偏），作为优化变量初始值
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);

        // optimize
        // 将当前变量和因子图加入优化器中，进行优化，优化结束后清空因子图和变量
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();

        // Overwrite the beginning of the preintegration for the next step.
        // 计算优化后结果
        gtsam::Values result = optimizer.calculateEstimate();

        // 获取优化后的状态（包括位姿、速度、零偏）
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));

        // Reset the optimization preintegration object.
        // 用优化后的零偏来重置 IMU 预积分接口，作为下一个 IMU 预积分的接口
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        // check optimization
        // 进行失效检测，如果判断当前估计失效则重置优化器及其相关变量，等待下一次初始化
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        // 对当前 Lidar 关键帧时间和最新收到的 IMU 消息时间之间的 IMU 消息进行预积分计算

        // 获得当前 Lidar 关键帧时间对应的状态
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        // 清理早于当前关键帧时间的 IMU 消息
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // 对当前 Lidar 关键帧时间和最新收到的 IMU 时间之间的 IMU 消息进行预积分计算

            // reset bias use the newly optimized bias
            // 用优化后的零偏重置 IMU 预积分接口
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    ///
    ///@brief 通过当前速度和零偏判断状态估计是否失效
    ///
    ///@param velCur 
    ///@param biasCur 
    ///@return true -- 失效 
    ///@return false -- 没有失效
    ///
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        // 速度太大 > 30 m/s (or 108 km /s)
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        // 零偏太大 || b || > 1m/s^2 or 1 rad / s
        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        // 旋转 IMU 消息使其和 Lidar 坐标系各轴平行
        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        // Opt 队列用于执行 IMU 预积分和优化，Imu 队列用于更新最新 IMU 状态
        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false)
            return;

        // 计算上一次 IMU 发布位姿的时刻和当前时刻的时间差
        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        // integrate this single imu message
        // 将当前 IMU 消息加入预积分接口中，预积分接口包含上一个 Lidar 关键帧到当前时刻的所有 IMU 消息的预积分结果
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

        // predict odometry
        // 根据上一关键帧的位姿估计和预积分观测预测当前时刻的里程计位姿
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        // 位姿发布
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");

    // IMU 预积分    
    IMUPreintegration ImuP;

    // 位姿融合
    TransformFusion TF;

    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}
