#pragma once
#include <iostream>
#include <map>
#include <random>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include "Estimator.hpp"

namespace vio{
class ScenePointCloud{
 public:
  std::vector<cv::Vec3f> cloudPoints_; 
  bool updateFlg_;
  cv::viz::Color color_;
  int displaySize_;

  ScenePointCloud() {
  }

  ScenePointCloud(const std::vector<cv::Vec3f>& cloudPts,
    cv::viz::Color color = cv::viz::Color::white(),
    int size = 2) {
    cloudPoints_ = cloudPts;
    updateFlg_ = true;
    color_ = color;
    displaySize_ = size;
  }
  ~ScenePointCloud() {
    cloudPoints_.clear();
  }
  void update(const std::vector<cv::Vec3f>& pts3D) {
    updateFlg_ = true;
    cloudPoints_.clear();
    cloudPoints_ = pts3D;
  }
};
class CameraObject {
 public:
  cv::viz::WCameraPosition cameraCoordinateFrame_; //相机坐标系模型
  cv::viz::WCameraPosition cameraFrustum_; //相机锥体模型
  std::vector<cv::Affine3d> path_; //相机位置path
  cv::Affine3d cameraPose_; //camera当前位置
  std::string cCoorScalarName_,cFrusName_,cPathName_; //相关名称
  bool poseUpdateFlg_;
  /** \brief constructor
   */ 
  CameraObject(){
  };

  /** \brief constructor
   * @param cameraName - name of camera
   * @param coorScalar - scalar of coordinate 
   * @param frustumScalar - scalar of camera frustum 
   * @param fov - field of view of the camera (horizontal angle,vertical angle)]
   * @param pos - pose of camera centre
   * @param focalPointPos - focal point pose
   * @param yDirection - y axis direction of camera(确定的y轴是该向量在由焦点和光心确定法向量对应平面上的投影)
   */ 
  CameraObject(std::string cameraName,
               float coorScalar,
               float frustumScalar,
               cv::Vec2f fov,
               cv::Vec3f pos,
               cv::Vec3f focalPointPos,
               cv::Vec3f yDirection,
               float recordPathFreq):
    cameraCoordinateFrame_(coorScalar),
    cameraFrustum_(fov,frustumScalar,cv::viz::Color::green()) {
    //由相机中心位置、焦点位置和y轴方向确定相机位姿
    cameraPose_ = cv::viz::makeCameraPose(pos,focalPointPos,yDirection);
    cCoorScalarName_ = cameraName + "_coor";
    cFrusName_ = cameraName + "_frustum";
    cPathName_ = cameraName + "_path";
    path_.clear();
    recoreFreq_ = recordPathFreq;
    recordCount_ = 0;
    poseUpdateFlg_ = true;
  };

  void updatePose(const cv::Affine3d& pos){
    cameraPose_ = pos;
    cameraCoordinateFrame_.setPose(cameraPose_);
    cameraFrustum_.setPose(cameraPose_);
    recordCount_++;
    if(recordCount_ % recoreFreq_ == 0) {
      path_.push_back(pos);
      if (path_.size() > 3000) {
        path_.erase(path_.begin());
      }
    }
    poseUpdateFlg_ = true;
  };
  void clearPath() {
    path_.clear();
    recordCount_ = 0;
  }
 private:
    int recoreFreq_;
    int recordCount_;
};
class VizScene {
 public:
  std::map<std::string,ScenePointCloud> sceneCloud_;
  std::map<std::string,CameraObject> sceneCamera_;
  cv::viz::Viz3d* sceneWindowPtr_;
  std::thread* windowLoopThread_;
  std::mutex mCloud_;
  const Estimator* estimator_;
  /** \brief construction function
   */ 
  VizScene(std::string windowName,double scale = 1.0);

  VizScene(const Estimator* estimator, double scale);

  /** \brief disconstruction function
   */ 
  ~VizScene();

  /** \brief viz window show at spinOnce way
   */
  void windowShowLoopRun();
  
  /** \brief show pointclouds in parameter sceneCloud_
   */
  void showSceneAllPointClouds();

  /** \brief show all camera objects in scene
   */ 
  void showSceneAllCamera();

  /** \brief create pointclouds from 3D point 
   * @param ptsName - name of point cloud
   * @param pts3D - 3D points vector
   * @param color - display point color
   * @param displaySize - display point size
   */ 
  bool createPointClouds(std::string ptsName,std::vector<cv::Vec3f>& pts3D,cv::viz::Color color = cv::viz::Color::white(),int displaySize = 2);

  /** \brief create camera object  
   * @param cameraName - camera name   
   * @param coorScalar - scalar of camera coordinate
   * @param frustumScalar - scalar of camera frustum(相机四方锥体模型) 
   * @param fov - field of view of the camera<horizontal angle,vertical angle>
   * @param pos - pose of camera centre
   * @param focalPointPos - focal point position of camera
   * @param yDirection -  y axis direction of camera frame(确定的y轴是该向量在由焦点和光心确定法向量对应平面上的投影)
   */
  bool createCameraObject(std::string cameraName,float coorScalar,float frustumScalar,cv::Vec2f fov,cv::Vec3f pos,cv::Vec3f focalPointPos,cv::Vec3f yDirection);
  
  /** \brief update pointcloud positions which named as ptsname
   * @param ptsName - name of pointcloud
   * @param pts3D - pointcloud
   */ 
  bool updatePointClouds(std::string ptsName,const std::vector<cv::Vec3f>& pts3D); 

  /** \brief update pose of camera named by "cameraName"
   * @param cameraName - camera name
   * @param cameraPose - camera pose 
   */
  bool updateCameraPose(std::string cameraName,const cv::Affine3d& cameraPose);

  /** \brief clear path of camera
   * @param cameraName  --- camera name in the vizscene
   */
  void clearCameraPath(std::string cameraName);

  /** \brief clear pointcoud
   * @param pcName --- the name of pointcloud
   */
  void clearPointCloud(std::string pcName);
};
}