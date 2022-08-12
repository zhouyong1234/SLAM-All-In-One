#pragma once
#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"

namespace vio{
struct OptitrackParam {
  float minDist;
  float qualityLevel;
  int maxPoints; 
  int trackBack;
  int predictEnable;
  int trackBackPixelErr;
  double pyramidLevel;
  int iterations;
  double eps;
  int showTrackFrames;
  int showDebugInfos;
  void print() {
    std::cout << "========OptiTrack Parameters========" << std::endl;
    std::cout << "trackBack = " << trackBack << std::endl;
    std::cout << "predictEnable = " << predictEnable << std::endl;
    std::cout << "trackBackPixelErr = " << trackBackPixelErr << std::endl;
    std::cout << "qualityLevel = " << qualityLevel << std::endl;
    std::cout << "maxPoints = " << maxPoints << std::endl;
    std::cout << "minDist = " << minDist << std::endl;
    std::cout << "pyramidLevel = " << pyramidLevel << std::endl;
    std::cout << "iterations = " << iterations << std::endl;
    std::cout << "eps = " << eps << std::endl;
    std::cout << "showTrackFrames = " << showTrackFrames << std::endl;
    std::cout << "showDebugInfos = " << showDebugInfos << std::endl;
  }
};
struct CameraParam {
  cv::Mat K;
  cv::Mat D;
  int is_fisheye;
  int width,height;
  void print() {
    std::cout << "=========Camera Parameters=========" << std::endl;
    std::cout << "IsFisheye = " << is_fisheye << std::endl;
    std::cout << "Width = " << width << std::endl;
    std::cout << "Height = " << height << std::endl;
    std::cout << "Intrinisic Matrix : \n" << K << std::endl;
    std::cout << "Distortion Matrix : \n" << D << std::endl;
  }
};
struct ExtrinsicParam {
  cv::Mat Rbc;
  cv::Mat tbc;
  void print() {
    std::cout << "=========Extrinsic Parameters=========" << std::endl;
    std::cout << "Rbc = \n" << Rbc << std::endl;
    std::cout << "tbc = " << tbc.t() << std::endl;
  }
};

struct InitialParam {
  float minDisparity;
  int minMatchedFeatNum;
  float reprojectErr;
  float homographyTransformErr;
  void print() {
    std::cout << "=========Initializator Parameters=========" << std::endl;
    std::cout << "minDisparity = " << minDisparity << std::endl;
    std::cout << "minMatchedFeatNum = " << minMatchedFeatNum << std::endl;
    std::cout << "reprojectErr = " << reprojectErr << std::endl;
    std::cout << "homographyTransformErr = " << homographyTransformErr << std::endl;
  }
};

struct PnpSolverParam {
  float reprojectErr;
  float successRatio;
  int showDebugInfo;
  void print() {
    std::cout << "==========PnpSolver Parameters==========" << std::endl;
    std::cout << "reprojectErr = " << reprojectErr << std::endl;
    std::cout << "successRatio = " << successRatio << std::endl;
    std::cout << "showDebugInfo = " << showDebugInfo << std::endl;
  }
};

struct SimulatorParam {
  cv::Vec3f length;
  cv::Vec3f origin;
  int featureSize;
  void print() {
    std::cout << "=========Simulator Parameters==========" << std::endl;
    std::cout << "x_length = " << length[0] << std::endl;
    std::cout << "y_length = " << length[1] << std::endl;
    std::cout << "z_length = " << length[2] << std::endl;
    std::cout << "x_origin = " << origin[0] << std::endl;
    std::cout << "y_origin = " << origin[1] << std::endl;
    std::cout << "z_origin = " << origin[2] << std::endl;
    std::cout << "featureSize = " << featureSize << std::endl;
  }
};

struct EstimatorParam {
  int KeyFrameParallexThreshold;
  int KeyFrameMatchedPoints;
  int ReprojectPixelErr;
  int BundleAdjustment;
  int EnableFeaturePrediction;
  int EnableVOAlignedToWorld;
  std::string LogName;
  void print() {
    std::cout << "=========Estimator Parameters========" << std::endl;
    std::cout << "keyframeParallexThreshold = " << KeyFrameParallexThreshold << std::endl;
    std::cout << "keyframeMatchedPoints = " << KeyFrameMatchedPoints << std::endl;
    std::cout << "reprojectPixelErr = " << ReprojectPixelErr << std::endl;
    std::cout << "bundleAdjustment = " << BundleAdjustment << std::endl;
    std::cout << "enableFeaturePrediction = " << EnableFeaturePrediction << std::endl;
    std::cout << "enableVOAlignedToWorld = " << EnableVOAlignedToWorld << std::endl;
    std::cout << "logName = " << LogName << std::endl;
  }
};

class Config {
 public:

  /** \brief construct function
   */
  Config(std::string configFile){
    cv::FileStorage fs(configFile,cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "[Config]:Config file is not exist!" << std::endl;
      return;
    }

    estimatorParam_.KeyFrameMatchedPoints = fs["Estimator.KeyFrameMatchedPoints"];
    estimatorParam_.KeyFrameParallexThreshold = fs["Estimator.KeyFrameParallexThreshold"];
    estimatorParam_.ReprojectPixelErr = fs["Estimator.ReprojectPixelErr"];
    estimatorParam_.BundleAdjustment = fs["Estimator.BundleAdjustment"];
    estimatorParam_.EnableFeaturePrediction = fs["Estimator.EnableFeaturePrediction"];
    estimatorParam_.EnableVOAlignedToWorld = fs["Estimator.EnableVOAlignedToWorld"];
    fs["Estimator.FileName"] >> estimatorParam_.LogName;
    if (estimatorParam_.LogName.empty()) {
      estimatorParam_.LogName = "estimator.csv";
    }
    estimatorParam_.print();

    optParam_.trackBack = fs["OptiTrack.TrackBack"];
    optParam_.predictEnable = fs["OptiTrack.PredictEnable"];
    optParam_.trackBackPixelErr = fs["OptiTrack.TrackBackPixelErr"];
    optParam_.maxPoints = fs["OptiTrack.MaxPoints"];
    optParam_.minDist = fs["OptiTrack.MinDist"];
    optParam_.qualityLevel = fs["OptiTrack.QualityLevel"];
    optParam_.pyramidLevel = fs["OptiTrack.PyramidLevel"];
    optParam_.iterations = fs["OptiTrack.Iterations"];
    optParam_.eps = fs["OptiTrack.EPS"];
    optParam_.showTrackFrames = fs["OptiTrack.ShowTrackFrames"];
    optParam_.showDebugInfos = fs["OptiTrack.ShowDebugInfos"];
    optParam_.print();

    fs["Camera.Intrinisic"] >> camParam_.K;
    fs["Camera.Distortion"] >> camParam_.D;
    camParam_.is_fisheye = fs["Camera.IsFisheye"]; 
    camParam_.width = fs["Camera.Width"];
    camParam_.height = fs["Camera.Height"];
    camParam_.print();

    fs["Rbc"] >> extrinsicParam_.Rbc;
    fs["tbc"] >> extrinsicParam_.tbc;
    extrinsicParam_.print();

    iniParam_.minDisparity = fs["Init.MinDisparity"];
    iniParam_.minMatchedFeatNum = fs["Init.MinMatchedFeatNum"];
    iniParam_.reprojectErr = fs["Init.ReprojectErr"];
    iniParam_.homographyTransformErr = fs["Init.HomographyTransformErr"];
    iniParam_.print();

    pnpParam_.reprojectErr = fs["PnpSolver.ReprojectErr"];
    pnpParam_.successRatio = fs["PnpSolver.SuccessRatio"];
    pnpParam_.showDebugInfo = fs["PnpSolver.ShowDebugInfo"];
    pnpParam_.print();

    simParam_.length[0] = fs["Sim.Length.x"];
    simParam_.length[1] = fs["Sim.Length.y"];
    simParam_.length[2] = fs["Sim.Length.z"];
    simParam_.origin[0] = fs["Sim.Origin.x"];
    simParam_.origin[1] = fs["Sim.Origin.y"];
    simParam_.origin[2] = fs["Sim.Origin.z"];
    simParam_.featureSize = fs["Sim.FeatureSize"];
    simParam_.print();


      
  };
  OptitrackParam optParam_;
  CameraParam camParam_;
  InitialParam iniParam_;
  SimulatorParam simParam_;
  PnpSolverParam pnpParam_;
  ExtrinsicParam extrinsicParam_;
  EstimatorParam estimatorParam_;
};

}