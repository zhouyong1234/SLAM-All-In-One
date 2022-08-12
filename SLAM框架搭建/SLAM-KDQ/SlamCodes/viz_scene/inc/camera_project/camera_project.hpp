#pragma once
#include <fstream>
#include <random>
#include "camodocal/camera_models/CameraFactory.h"
#include "utilities/access_file.hpp"

using namespace camodocal;
class CameraProject {
  private:
    bool inBorder(const cv::Point2i& pt);
    std::string outPtsFile_;
    float pixelNoise_;
    bool addNoise_;
  public:

    /** @brief Constructor
     *  @param addNose - 1:add pixel noise 0: no noise
     *  @param pixelNoise - noise sigma 
     *  @param cameraConfigFile - config file about camera which is used to create camera object
     *  @param outputPtsFile - output file for record infos about project
     */ 
    CameraProject(bool addNoise,float pixelNoise,std::string cameraConfigFile,std::string outputPtsFile);

    /** @brief Destructor
     * 
     */ 
    ~CameraProject();

    /** @brief project 3D points from world to image plane
     *  @param t - timestamp
     *  @param ptsCloud - pointclouds object type is cv::Mat(1,pts.size(),CV_32FC3);
     *  @param cameraPos - pose of camera in world frame
     */ 
    ProjectPointInfo projectVizPoints(double t,const std::vector<cv::Vec3f>& ptsCloud,const Eigen::Isometry3d& cameraPos);

    /** @brief project pixel point from image plane to world
     *  @param cameraPos - pose of camera in world frame
     *  @param uv - pixel coordinate
     *  @param scale - distance between 3D point and camera 
     */ 
    Eigen::Vector3d pixelInverseProjectToWorld(const Eigen::Isometry3d& cameraPos,Eigen::Vector2d uv,double scale);

    CameraPtr camPtr_;
};