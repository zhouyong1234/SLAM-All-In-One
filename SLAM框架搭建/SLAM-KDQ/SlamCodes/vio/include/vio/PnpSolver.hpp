#pragma once
#include <opencv2/calib3d.hpp>
#include "Config.hpp"
#include "fileSystem.hpp"

namespace vio{
class PnpSolver {
 public:
  PnpSolver(const Config* cfg):
  ReprojectErr(cfg->pnpParam_.reprojectErr),
  SuccessRatio(cfg->pnpParam_.successRatio),
  ShowDebugInfo(cfg->pnpParam_.showDebugInfo) {

  };

  /** \brief Solve pose using pnp solver ransac method
   * @param normalizedUV  ---  normalized features vector  
   * @param matchedPts3D  ---  matched features' 3D vector
   * @param focalLength   ---  camera focal length
   * @param rcw           ---  rotation vec from world to camera coordinate
   * @param CtW           ---  translate vec from world to camera coordinate
   * @param inlier        ---  inlier vector of reproject 
   * @return return success otherwise false
   */ 
  bool solveByPnp(const std::vector<cv::Point2f>& normalizedUV,
                  const std::vector<cv::Point3f>& matchedPts3D,
                  double focalLength,
                  cv::Mat &rcw,
                  cv::Mat &CtW,
                  std::vector<int>& inliers) {
    if (normalizedUV.size() < 4) {
      return false;
    }
    std::vector<int> rawInliers;
    cv::Mat K = cv::Mat::eye(3,3,CV_64F);
    bool resultFlg = cv::solvePnPRansac(matchedPts3D,normalizedUV,K,cv::Mat(),rcw,CtW, true,100,ReprojectErr/focalLength,0.9,inliers);
    if (ShowDebugInfo) {
      for (int i = 0; i < matchedPts3D.size(); i++) {
        cv::Affine3d T(rcw,CtW);
        cv::Point3f ptInCam =  T * matchedPts3D[i];
        cv::Point2f ptInCamUV(ptInCam.x / ptInCam.z, ptInCam.y / ptInCam.z);
        cv::Point2f reproErr = normalizedUV[i] - ptInCamUV;
        int isInlier = cv::norm(reproErr) < ReprojectErr / focalLength;
        std::cout << " Inlier: "<< isInlier << ", " << ptInCamUV << " vs " << normalizedUV[i] << " vs " << cv::norm(reproErr) << std::endl;
      }
    }
    FileSystem::printInfos(LogType::Info,"PnpSolver","PnpSolver success %d,all features' size %d and inliers' size %d",resultFlg,matchedPts3D.size(),inliers.size()) ;
    return resultFlg && inliers.size() > SuccessRatio * normalizedUV.size();
  }
 private:
  const float ReprojectErr;
  const float SuccessRatio;
  int ShowDebugInfo;
};
}