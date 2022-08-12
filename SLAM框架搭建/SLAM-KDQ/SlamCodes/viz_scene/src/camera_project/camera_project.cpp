#include <map>
#include "camera_project/camera_project.hpp"

CameraProject::CameraProject(bool addNoise,float pixelNoise,std::string cameraConfigFile,std::string outputPtsFile) {
  if(cameraConfigFile.empty()) {
    camPtr_ = nullptr;
  } else {
    camPtr_ = CameraFactory::instance()->generateCameraFromYamlFile(cameraConfigFile);
    std::cout << "camera size : " << camPtr_->imageWidth()<< "," << camPtr_->imageHeight() << std::endl;
  }
  outPtsFile_ = outputPtsFile;
  pixelNoise_ = pixelNoise;
  addNoise_ = addNoise;
}
CameraProject::~CameraProject() {

}

ProjectPointInfo CameraProject::projectVizPoints(double t,const std::vector<cv::Vec3f>& ptsCloud,const Eigen::Isometry3d& Twc) {
  if(ptsCloud.empty()) {
    return ProjectPointInfo();
  }
  if (camPtr_ == nullptr) {
    std::cout << "camera is not create" << std::endl;
    return ProjectPointInfo();
  }
  std::mt19937 gen{12345};
  std::normal_distribution<> d{0.0, pixelNoise_};

  Eigen::Isometry3d Tcw = Twc.inverse();
  ProjectPointInfo ptsInfo;
  ptsInfo.Twc = Twc;
  ptsInfo.t = t;
  for (size_t i = 0; i < ptsCloud.size(); i++) {
    Eigen::Vector3d pW(ptsCloud[i][0],ptsCloud[i][1],ptsCloud[i][2]);
    Eigen::Vector3d pC = Tcw * pW;
    Eigen::Vector2d pixel;
    if (pC.z() < 0) {
      continue;
    }
    camPtr_->spaceToPlane(pC,pixel);
    cv::Point2i uv;
    uv.x = pixel.x() + 0.5 + (addNoise_? std::round(d(gen)) : 0.0);
    uv.y = pixel.y() + 0.5 + (addNoise_? std::round(d(gen)) : 0.0);
    if (inBorder(uv)) {
      ptsInfo.ptsMap[i] = std::make_pair(cv::Point3d(pW.x(),pW.y(),pW.z()),uv);
    }
  }
  VioDatasInterface::recordCameraPixel(ptsInfo,outPtsFile_);
  return ptsInfo;
}

bool CameraProject::inBorder(const cv::Point2i& pt)
{
    const int BORDER_SIZE = 1;
    int img_x = pt.x;
    int img_y = pt.y;
    return BORDER_SIZE <= img_x && img_x < camPtr_->imageWidth() - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < camPtr_->imageHeight() - BORDER_SIZE;
}

Eigen::Vector3d CameraProject::pixelInverseProjectToWorld(const Eigen::Isometry3d& cameraPos,Eigen::Vector2d uv,double scale) {
  Eigen::Vector3d p3C;
  camPtr_->liftSphere(uv,p3C);
  p3C = p3C * scale;
  Eigen::Vector3d p3W = cameraPos * p3C;
  return p3W;
}