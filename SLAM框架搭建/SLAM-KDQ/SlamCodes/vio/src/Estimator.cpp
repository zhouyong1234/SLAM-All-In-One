#include "Estimator.hpp"
#include <opencv2/core/eigen.hpp>
#include "BASolver.hpp"
#include "fileSystem.hpp"
#include "tictoc.hpp"

namespace vio{
Estimator::Estimator(std::string configFile) {
  cfg_ = new Config(configFile);
  cam_ = new Camera(cfg_);
  init_ = new Initializator(cfg_,cam_);
  feaTrcker_ = new FeatureTracker(cfg_);
  pnpSolver_ = new PnpSolver(cfg_);
  baSolver_ = new BAG2O();
  state = EstState::Waiting;
  preInteNow_ = nullptr;
  moduleName_ = "Estimator";
  cv::cv2eigen(cfg_->extrinsicParam_.Rbc,Rbc_);
  cv::cv2eigen(cfg_->extrinsicParam_.tbc,tbc_);
  if (cfg_->estimatorParam_.EnableVOAlignedToWorld) {
    alignWorld_ = new VOAlignedRealWorld(Rbc_.transpose(),500,30,3.,0.3);
  } else {
    alignWorld_ = nullptr;
  }
  FileSystem::fInfo = fopen(cfg_->estimatorParam_.LogName.c_str(),"wb+");
  FileSystem::printInfos(LogType::Info,moduleName_ + "|Pose","timestamp,allCost,trackCost,initCost,pnpCost,baCost,alignCost,px,py,pz,qw,qx,qy,qz");
  reset();
}

Estimator::~Estimator() {
  reset();
  delete cfg_;
  delete cam_;
  delete init_;
  delete feaTrcker_;
  delete pnpSolver_;
  if (alignWorld_ != nullptr)
    delete alignWorld_;
}

bool Estimator::getCurrentPose(Eigen::Vector3d &t, Eigen::Quaterniond &q) const {
  cv::Mat Rwc,twc;
  if (getCurrentPose(Rwc,twc)) {
    cv::cv2eigen(twc,t);
    Eigen::Matrix3d R;
    cv::cv2eigen(Rwc,R);
    q = Eigen::Quaterniond(R).normalized();
    return true;
  }
  return false;
}

void Estimator::update(FramePtr frame,bool trackEnable) {
  std::lock_guard<std::mutex> lock(m_filter_);
  AdvanceTimer timer1,timer2;
  FramePtr lastFramePtr = nullptr;
  if (!slideWindows_.empty()) {
    lastFramePtr = slideWindows_.back();
  }
  if (trackEnable) {
    cv::Mat R_cur_last;
    if (lastFramePtr != nullptr && cfg_->estimatorParam_.EnableFeaturePrediction == 1)
      calCameraRotationMatrix(lastFramePtr->timestamp_,frame->timestamp_,R_cur_last);
    feaTrcker_->detectAndTrackFeature(lastFramePtr, frame, R_cur_last);
  }
  timer1.toc("trackCost");
  isKeyframe(frame);
  slideWindows_.push_back(frame);
  poseUpdateFlg_ = false;
  switch (state)
  {
    case EstState::Waiting:
    {
      if (slideWindows_.size() >= 2) {
        float parallex = 0.;
        FramePtr refFrame = slideWindows_.front();
        //KDQ:初始化的时候第一帧选择很重要，如果第一帧和当前帧匹配带你过少，则应该删除第一帧
        while (!slideWindows_.empty() && refFrame->getMatchedFeatureSize(frame.get(), parallex) < cfg_->iniParam_.minMatchedFeatNum) {
          slideWindows_.erase(slideWindows_.begin());
        }
        if (slideWindows_.size() > 4 && parallex > cfg_->iniParam_.minDisparity) {
          state = EstState::Initing;
        }
      }
      break;
    }
    case EstState::Initing:
    {
      FramePtr refFrame = slideWindows_.front();
      size_t endId = slideWindows_.size() - 1;
      int initCnt = 1;
      if (init_->initPoseAndMap(slideWindows_[0],slideWindows_[endId],fsm_)) {
        for (;initCnt < endId;initCnt++) {
          if (estimatePose(slideWindows_[initCnt])) {
            updateFeature(slideWindows_[initCnt]);
          } else {
            break;
          }
        }
      } else {
        break;
      }
      if (initCnt == endId) {
        //三角化所有的特征点
        state = Runing;
//        fsm_.triangulateAll();
//        BAG2O basolver;
//        if (basolver.constructWindowFrameOptimize(slideWindows_,fsm_,2.0/cam_->fx())) {
//          basolver.updatePoseAndMap(slideWindows_,fsm_);
//          state = Runing;
//        } else {
//          reset();
//          state = Waiting;
//        }
      } else {
        reset();
        state = Waiting;
      }
      timer1.toc("initCost");
      break;
    }
    case EstState::Runing:
      if (slideWindows_.size() > 1 && fsm_.getFeatureSize() > 5 && estimatePose(slideWindows_.back()) && checkPose()) {
        //updateFeature must be first than ba,otherwize curframe not be update
        updateFeature(slideWindows_.back());
        timer1.toc("pnpCost");
        if (alignWorld_ != nullptr && alignUpdateFlg_) {
          alignWorld_->alignToRealWorld();
          alignUpdateFlg_ = false;
        }
        timer1.toc("alignCost");
        bundleAdjustment();
        timer1.toc("baCost");
        poseUpdateFlg_ = true;
      } else {
        reset();
        state = EstState::Waiting;
      }
      break;
    default:
      break;
  }
  slideWindow();
  if (slideWindows_.empty()) {
    return;
  }
  timer2.toc("allCost");
  Eigen::Vector3d twc = Eigen::Vector3d::Zero();
  Eigen::Quaterniond qwc;
  qwc.setIdentity();
  getCurrentPose(twc,qwc);
  FileSystem::printInfos(LogType::Info,moduleName_ + "|Pose","%12.4f,%d,"
                                                             "%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,"
                                                             "%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,%3.4f",
                         slideWindows_.back()->timestamp_,state,
                         timer2.costMs("allCost"),timer1.costMs("trackCost"), timer1.costMs("initCost"),timer1.costMs("pnpCost"),timer1.costMs("baCost"),timer1.costMs("alignCost"),
                         twc.x(),twc.y(),twc.z(),qwc.w(),qwc.x(),qwc.y(),qwc.z());
}

void Estimator::slideWindow() {
  if (slideWindows_.size() > WINSIZE ) { //remove old frame until slidewindow is full
    //删除最老帧
    if (removeOldKeyFrame_) {
      slideWindows_.front()->image_.release();
      slideWindows_.erase(slideWindows_.begin());
    } else {
      //删除次新帧
      FramePtr curFrame = slideWindows_.back();
      slideWindows_.pop_back();
      FramePtr secondNewFrame = slideWindows_.back();
      secondNewFrame->image_.release();
      slideWindows_.pop_back();
      slideWindows_.push_back(curFrame);
    }
  }
}

void Estimator::reset() {
  if (alignWorld_ != nullptr)
    alignWorld_->reset();
  fsm_.reset();
  slideWindows_.clear();
  imuMeas_.clear();
  feaTrcker_->reset();
  poseUpdateFlg_ = false;
  removeOldKeyFrame_ = true;
  alignUpdateFlg_ = false;
}

void Estimator::isKeyframe(FramePtr frame) {
  if (slideWindows_.empty()) {
    return;
  }
  std::vector<uint64_t> idVec;
  std::vector<cv::Point2f> refFeatures,curFeatures;
  float averParallex = 0;
  frame->getMatchedFeatures(slideWindows_.back().get(),idVec,refFeatures,curFeatures,averParallex);
  if (curFeatures.size() < cfg_->estimatorParam_.KeyFrameMatchedPoints ||
      averParallex > cfg_->estimatorParam_.KeyFrameParallexThreshold) {
    removeOldKeyFrame_ = true;
  } else {
    removeOldKeyFrame_ = false;
  }
}

void Estimator::bundleAdjustment() {
  if (slideWindows_.size() < WINSIZE || !cfg_->estimatorParam_.BundleAdjustment) {
    return;
  }
  if (baSolver_->constructWindowFrameOptimize(slideWindows_,fsm_,2.0/cam_->fx())) {
    baSolver_->updatePoseAndMap(slideWindows_, fsm_);
  }
}

bool Estimator::estimatePose(FramePtr frame) {
  std::vector<cv::Point2f> matchedNormalizedUV;
  std::vector<cv::Point3f> matchedPts3D;
  std::vector<uint64_t> matchedIds;
  fsm_.featureMatching(frame,matchedIds,matchedNormalizedUV,matchedPts3D);
  if (matchedPts3D.size() < 5) {
    FileSystem::printInfos(LogType::Error,moduleName_ + "|EstimatePose","Matched points is too few less than 5!");
    return false;
  }
  cv::Mat rcw,CtW,Rcw;
  slideWindows_[slideWindows_.size()-2]->getInversePose(rcw,CtW);
  std::vector<int> inliers;
  if (pnpSolver_->solveByPnp(matchedNormalizedUV,matchedPts3D,cam_->fx(),rcw,CtW,inliers)) {
    cv::Mat Rwc,WtC;
    cv::Rodrigues(rcw,Rcw);
    Rwc = Rcw.t();
    WtC = - Rwc * CtW;
    frame->setPoseInWorld(Rwc,WtC);
    return true;
  }
  FileSystem::printInfos(LogType::Error,moduleName_ + "|EstimatePose","PnPSolver failure!");
  return false;
}


bool Estimator::checkPose() {
  static double dPose = -1.0;
  if (slideWindows_.size() < 2) {
    return true;
  }
  FramePtr curFramePtr = slideWindows_.back();
  FramePtr oldestFramePtr = slideWindows_.front();
  double dt = curFramePtr->timestamp_ - oldestFramePtr->timestamp_;
  Eigen::Vector3d cur_wtc = curFramePtr->EWtC();
  Eigen::Vector3d old_wtc = oldestFramePtr->EWtC();
  double nowDPose = (cur_wtc - old_wtc).norm();
  if (dPose > 0) {
    return nowDPose < 10 * dPose;
  }
  dPose = nowDPose;
  return true;
}

void Estimator::updateFeature(FramePtr curFramePtr) {
  //step1: add new features
  std::map<uint64_t,cv::Point2f> &corners = curFramePtr->getCorners();
  std::map<uint64_t,Feature>& features = fsm_.getFeatureMap();
  std::vector<uint64_t> idx;
  std::vector<cv::Point2f> ptVec,proPtVec;
  std::vector<cv::Point3f> p3DVec;
  for (auto it = corners.begin();it != corners.end(); it++) {
    const uint64_t id = it->first;
    cv::Point2f pt = it->second;
    if (features.count(id) && features[id].valid3D()) {
      ptVec.push_back(pt);
      p3DVec.push_back(features[id].getPts3DInWorld());
      idx.push_back(id);
    } else {
      fsm_.addFeature(id,curFramePtr);
    }
  }
  cv::Mat Rcw,CtW,rcw;
  curFramePtr->getInversePose(Rcw,CtW);
  cv::Rodrigues(Rcw,rcw);
  cam_->project(p3DVec,proPtVec,rcw,CtW);
  for(size_t i = 0; i < idx.size(); i++) {
    float repErr = cv::norm(proPtVec[i] - ptVec[i]);
    if (repErr > 5 * cfg_->estimatorParam_.ReprojectPixelErr) {
      fsm_.removeFeature(idx[i]);
    } else if (repErr > cfg_->estimatorParam_.ReprojectPixelErr) {
      fsm_.updateBadCount(idx[i]);
    } else {
      fsm_.addFeature(idx[i],curFramePtr);
      if (repErr < 0.5) {
        fsm_.updateGoodCount(idx[i]);
      }
    }
  }

  //step2: remove untracked and bad features
  for (auto it = features.begin(); it != features.end();) {
    if (slideWindows_.size() == WINSIZE && !it->second.isInFrame(slideWindows_.back())) {
      fsm_.removeFeature(it++);
      continue;
    }
    if (it->second.getTrackCount() == 0) {
      fsm_.removeFeature(it++);
      continue;
    }
    if (it->second.getBadCount() >= 3) {
      cv::Point3f pt3d = it->second.getPts3DInWorld();
      FileSystem::printInfos(LogType::Warning,moduleName_ + "|UpdateMap","Remove %lu feature[%f,%f,%f] for bad count > 3!\n",it->first,pt3d.x,pt3d.y,pt3d.z);
      fsm_.removeFeature(it++);
      continue;
    }
    it++;
  }
}

void Estimator::calCameraRotationMatrix(double lastT, double curT, cv::Mat &R_cur_last) {
  if (lastT > curT || curT - lastT > 1.0) {
    FileSystem::printInfos(LogType::Warning,moduleName_ + "|calCameraRotationMatrix","[calCameraRotationMatrix]:last timestamp: %12.4f and current timestamp: %12.4f have big interval or bad sequence!",lastT,curT);
    return;
  }
  std::lock_guard<std::mutex> imuLck(m_imu_);
  std::map<double,IMU>::const_iterator itBegin = imuMeas_.getLowIter(lastT);
  std::map<double,IMU>::const_iterator itEnd = imuMeas_.getHighIter(curT);

  if (itBegin == imuMeas_.measMap_.end() || itEnd == imuMeas_.measMap_.end()) {
    FileSystem::printInfos(LogType::Warning,moduleName_ + "|calCameraRotationMatrix","Imu infos not includes data from %12.4f to %12.4f!",lastT,curT);
    return;
  }
  IMU lastImu;
  if (preInteNow_ != nullptr)
    delete preInteNow_;
  preInteNow_ = new PreIntegration(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
  std::map<double,IMU>::const_iterator it = itBegin,lastIt = itBegin;
  while (it != itEnd) {
    IMU imu(it->second.timestamp_,it->second.acc_,it->second.gyro_);
    if (it != itBegin) {
      preInteNow_->midIntegration(imu);
    }
    lastImu = imu;
    lastIt = it;
    it++;
    if (lastIt == itBegin ) {
      IMU nexImu(it->second.timestamp_,it->second.acc_,it->second.gyro_);
      Eigen::Vector3d acc = (lastImu.acc_ * (lastT - lastImu.timestamp_) + nexImu.acc_ * (nexImu.timestamp_ - lastImu.timestamp_)) / (nexImu.timestamp_ - lastT);
      Eigen::Vector3d gyr = (lastImu.gyro_ * (lastT - lastImu.timestamp_) + nexImu.gyro_ * (nexImu.timestamp_ - lastImu.timestamp_)) / (nexImu.timestamp_ - lastT);
      IMU nowImu(lastT,acc,gyr);
      preInteNow_->midIntegration(nowImu);
    } else if (it == itEnd) {
      IMU nexImu(it->second.timestamp_,it->second.acc_,it->second.gyro_);
      Eigen::Vector3d acc = (lastImu.acc_ * (curT - lastImu.timestamp_) + nexImu.acc_ * (nexImu.timestamp_ - curT)) / (nexImu.timestamp_ - lastImu.timestamp_);
      Eigen::Vector3d gyr = (lastImu.gyro_ * (curT - lastImu.timestamp_) + nexImu.gyro_ * (nexImu.timestamp_ - curT)) / (nexImu.timestamp_ - lastImu.timestamp_);
      IMU nowImu(curT,acc,gyr);
      preInteNow_->midIntegration(nowImu);
    }
  }
  preInteNow_->fix();
  Eigen::Matrix3d R_lastB_curB = preInteNow_->rotationMatrix();
  Eigen::Matrix3d R = (R_lastB_curB * Rbc_).transpose() * Rbc_;
  cv::Mat cvR = (cv::Mat_<float>(3,3) << R(0,0), R(0,1), R(0,2),
    R(1,0), R(1,1), R(1,2),
    R(2,0), R(2,1), R(2,2));
  cvR.copyTo(R_cur_last);
}


}

