#pragma once
#include <fstream>
#include "FeatureManager.hpp"
#include "Initializator.hpp"
#include "Config.hpp"
#include "PnpSolver.hpp"
#include "FeatureTracker.hpp"
#include "MeasurementTimeline.hpp"
#include "PreIntegration.hpp"
#include "BASolver.hpp"
#include "AlignRealWorld.hpp"

namespace vio{

enum EstState {
  Waiting,
  Initing,
  Runing
};

class Estimator {
 public:
  std::string cameraName = "cam";
  std::string pointsName = "corners";

  /** \brief construct function
   * @param configFile --- config file
   */
  Estimator(std::string configFile);

  /** \brief deconstruct function
   */
  ~Estimator();

  /** \brief reset system
   */
  void reset();

  /** \brief estimator update 
   * @param FramePtr    --- frame shared_ptr 
   * @param trackEnable --- enable track features
   */
  void update(FramePtr frame,bool trackEnable);

  /** \brief get estimator state
   */
  inline EstState getEstimatorState() const{
    return state;
  }

  /** \brief get current pose
   * @param Rwc --- rotation matrix from camera to world
   * @param WtC --- translation vector from camera to world
   * @return return pose success
   */
  bool getCurrentPose(cv::Mat& Rwc,cv::Mat& WtC) const {
    if (!slideWindows_.empty() && poseUpdateFlg_) {
      slideWindows_.back()->getPoseInWorld(Rwc,WtC);
      return true;
    }
    return false;
  }

  bool getCurrentPose(Eigen::Vector3d &t,Eigen::Quaterniond& q) const;

  /** \brief get vio timestamp
   * @return timestamp
   */
  double getVIOTimestamp() const {
    if (slideWindows_.empty()) {
      return  0;
    }
    return slideWindows_.back()->timestamp_;
  }

  /** \brief get all features coordinate in world 
   */
  std::vector<cv::Vec3f> getFeatsInWorld() const {
    return fsm_.getPointsInWorld();
  }

  /** \brief update imu data
   *
   * @param t --- timestamp of imu data
   * @param data --- imu data
   */
  void updateImuMeas(double t,const IMU & data) {
    static double lastTimestamp = 0;
    std::lock_guard<std::mutex> imuLock(m_imu_);
    imuMeas_.addMeas(t,data);
    if (!slideWindows_.empty()) {
      double frameTimestamp = slideWindows_.back()->timestamp_;
      imuMeas_.clean( frameTimestamp- 1.0);
      if (alignWorld_ != nullptr && lastTimestamp != frameTimestamp && state == Runing && !alignWorld_->frameAlreadyInsert(frameTimestamp) && frameTimestamp < t) {
        lastTimestamp = frameTimestamp;
        alignWorld_->update(slideWindows_.back(),imuMeas_);
        alignUpdateFlg_ = true;
      }
    } else {
      while (imuMeas_.measMap_.size() > 1000) {
        imuMeas_.measMap_.erase(imuMeas_.measMap_.begin());
      }
    }


  }

  /** \brief get camera shared pointer
   * @return
   */
  const CameraPtr getCameraPtr() const {
    return CameraPtr(cam_);
  }

  /** \brief perform bundle adjustment using g2o solver
   *
   */
  void bundleAdjustment();

 private:
  EstState state;
  Config* cfg_;
  Camera* cam_;
  Initializator* init_;
  FeatureTracker* feaTrcker_;
  FeatureManager fsm_;
  PnpSolver* pnpSolver_;
  BAG2O * baSolver_;
  VOAlignedRealWorld *alignWorld_;
  std::vector<FramePtr> slideWindows_;
  std::mutex m_filter_,m_imu_;
  MeasurementTimeline<IMU> imuMeas_;
  PreIntegration* preInteNow_;
  Eigen::Matrix3d Rbc_;
  Eigen::Vector3d tbc_;
  bool poseUpdateFlg_;
  bool removeOldKeyFrame_;
  bool alignUpdateFlg_;
  std::string moduleName_;
  /** \brief slide window to remove oldest frame and features
   */
  void slideWindow();

  /** \brief estimate pose of current frame
   * @return return true if estimator works well otherwise return false
   */
  bool estimatePose(FramePtr framePtr);

  /** \brief check pose ok or not
   * @return return true if current pose is ok
   */
  bool checkPose();

  /** \brief update features include removing untracked points and add new points
   */
  void updateFeature(FramePtr frame);

  /** \brief check frame is keyframe or not through check parallax between frame and slidewindow back frame
   * @param frame --- frame
   */
  void isKeyframe(FramePtr frame);

  /** \brief calculate rotation matrix from last time to current time
   *
   * @param lastT --- last frame timestamp
   * @param curT --- current frame timestamp
   * @param R_cur_last --- rotation matrix from last frame to current frame
   */
  void calCameraRotationMatrix(double lastT, double curT, cv::Mat &R_cur_last);

};
}
