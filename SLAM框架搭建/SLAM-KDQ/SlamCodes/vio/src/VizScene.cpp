#include "VizScene.hpp"

namespace vio{
VizScene::VizScene(std::string windowName,double scale) {
  sceneCloud_.clear();
  sceneWindowPtr_ = new cv::viz::Viz3d(windowName);
  sceneWindowPtr_->showWidget("widget coordinate",cv::viz::WCoordinateSystem(scale));
#ifndef __APPLE__
  //windowLoopThread_ = new std::thread(&VizScene::windowShowLoopRun,this);
#endif
  //windowLoopThread_->join();//如果主线程执行很快就结束，必须加join来强行插入进程，否则主线程很快直接运行结束，导致无法运行该线程
}

VizScene::VizScene(const Estimator* estimator,double scale):estimator_(estimator) {
  if (estimator_ == nullptr) {
    return;
  }
  std::vector<cv::Vec3f> pts3D;
  pts3D.emplace_back(0,0,0);
  pts3D.emplace_back(0,0,0);
  createCameraObject(estimator_->cameraName,0.05,0.05,cv::Vec2f(CV_PI/2.0,CV_PI/2.0),cv::Vec3f(0.0,0.0,0.0),cv::Vec3f(0.0,0.0,1.0),cv::Vec3f(0,1.0,0));
  createPointClouds(estimator_->pointsName,pts3D,cv::viz::Color::red(),4);
  sceneWindowPtr_ = new cv::viz::Viz3d("vio");
  sceneWindowPtr_->showWidget("widget coordinate",cv::viz::WCoordinateSystem(scale));
#ifndef __APPLE__
  windowLoopThread_ = new std::thread(&VizScene::windowShowLoopRun,this);
#endif
}

VizScene::~VizScene() {
  sceneCloud_.clear();
  sceneCamera_.clear();
  delete windowLoopThread_;
  delete sceneWindowPtr_;
}
void VizScene::windowShowLoopRun() {
  while (!sceneWindowPtr_->wasStopped()) {
    sceneWindowPtr_->spinOnce(1,false);
    cv::Mat Rwc,WtC;
    if (estimator_->getEstimatorState() == EstState::Runing) {
      if (estimator_->getCurrentPose(Rwc, WtC)) {
        cv::Affine3d Twc(Rwc, WtC);
        Eigen::Vector3d  t;
        Eigen::Quaterniond qwc;
        estimator_->getCurrentPose(t,qwc);
        kindr::RotationQuaternionPD kqwc(qwc);
        kindr::EulerAnglesZyxD euler(kqwc);
        euler.setUnique(); //设置唯一的转换方式，不然有可能变成很大的值
        Eigen::Vector3d eulerDeg = euler.toImplementation() * 180 / M_PI;
        std::cout << "[Pose]:" << estimator_->getVIOTimestamp() << "," <<  t.transpose() << " -- " << eulerDeg.transpose() << std::endl;
        updateCameraPose(estimator_->cameraName, Twc);
        updatePointClouds(estimator_->pointsName, estimator_->getFeatsInWorld());
      }
    } else {
      cv::Affine3d Twc;
      Twc.Identity();
      updateCameraPose(estimator_->cameraName, Twc);
      clearCameraPath(estimator_->cameraName);
      clearPointCloud(estimator_->pointsName);
      std::vector<cv::Vec3f> pts3D;
      pts3D.emplace_back(0,0,0);
      pts3D.emplace_back(0,0,0);
      createPointClouds(estimator_->pointsName,pts3D,cv::viz::Color::red(),4);
    }
    showSceneAllCamera();
    showSceneAllPointClouds();
    usleep(1000);
  }
}

void VizScene::showSceneAllPointClouds() {
  for (auto it = sceneCloud_.begin(); it != sceneCloud_.end();it++) {
    if(it->second.updateFlg_ == false) {
      continue;
    }
    cv::viz::WCloud widgetCloud(it->second.cloudPoints_,it->second.color_);
    widgetCloud.setRenderingProperty(cv::viz::POINT_SIZE,it->second.displaySize_);
    sceneWindowPtr_->showWidget(it->first,widgetCloud);
    it->second.updateFlg_ = false;
  }
}

void VizScene::showSceneAllCamera() {
  for (auto it = sceneCamera_.begin(); it != sceneCamera_.end(); it++) {
    if (it->second.poseUpdateFlg_ == false) {
      continue;
    }
    sceneWindowPtr_->showWidget(it->second.cCoorScalarName_,
                                it->second.cameraCoordinateFrame_,
                                it->second.cameraPose_);
    sceneWindowPtr_->showWidget(it->second.cFrusName_,
                                it->second.cameraFrustum_,
                                it->second.cameraPose_);
    cv::viz::WTrajectory cameraPath(it->second.path_);
    sceneWindowPtr_->showWidget(it->second.cPathName_,cameraPath);
    it->second.poseUpdateFlg_ = false;
  }
}

bool VizScene::createPointClouds(std::string ptsName,std::vector<cv::Vec3f>& pts3D,cv::viz::Color color,int displaySize) {
  if (pts3D.empty()) {
    printf("Input points vector is empty!\n");
    return false;
  }
  if (sceneCloud_.count(ptsName)) {
    printf("%s pointcloud is already created!\n",ptsName.c_str());
    return false;
  }
  ScenePointCloud pc(pts3D,color,displaySize);
  sceneCloud_[ptsName] = pc;
  return true;
}

bool VizScene::createCameraObject(std::string cameraName,
                                  float coorScalar,
                                  float frustumScalar,
                                  cv::Vec2f fov,
                                  cv::Vec3f pos,
                                  cv::Vec3f focalPointPos,
                                  cv::Vec3f yDirection) {
  if(sceneCamera_.count(cameraName)) {
    printf("camera %s is already create!\n",cameraName.c_str());
    return false;
  }
  CameraObject camera(cameraName,coorScalar,frustumScalar,fov,pos,focalPointPos,yDirection,3);
  sceneCamera_[cameraName] = camera;
  return true;
}

bool VizScene::updatePointClouds(std::string ptsName,const std::vector<cv::Vec3f>& pts) {
  std::lock_guard<std::mutex> lockWCloud(mCloud_);
  if (!sceneCloud_.count(ptsName)) {
    printf("No %s pointcloud exist!\n",ptsName.c_str());
    return false;
  }
  if (pts.empty()) {
    printf("PointCloud empty!\n");
    return false;
  }
  sceneCloud_[ptsName].update(pts);
  return true;
}

bool VizScene::updateCameraPose(std::string cameraName,const cv::Affine3d& cameraPose) {
  std::lock_guard<std::mutex> lockWCloud(mCloud_);
  if(!sceneCamera_.count(cameraName)) {
    printf("Camera %s is not exist!\n",cameraName.c_str());
    return false;
  }
  sceneCamera_[cameraName].updatePose(cameraPose);
  return true;
}

void VizScene::clearCameraPath(std::string cameraName) {
  if (sceneCamera_.count(cameraName)) {
    sceneCamera_[cameraName].path_.clear();
  }
}

void VizScene::clearPointCloud(std::string pcName) {
  if (sceneCloud_.count(pcName)) {
    sceneCloud_.clear();
  }
}


}



