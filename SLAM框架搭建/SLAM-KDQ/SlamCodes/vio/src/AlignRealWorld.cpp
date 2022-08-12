//
// Created by kdq on 2021/6/23.
//
#include "AlignRealWorld.hpp"

VOAlignedRealWorld::VOAlignedRealWorld(Eigen::Matrix3d Rcb,int imuFreq, int imageFreq, double alignPeriod, double alpha,double exciThr,bool enableGaussFilter) {
  Rcb_ = Rcb;
  enableGaussFilter_ = enableGaussFilter;
  imageDiv_ = (alignPeriod * imageFreq) / S_;
  if (imageDiv_ < 1) {
    imageDiv_ = 1;
  } else if (imageDiv_ > 6) {
    imageDiv_ = 6;
  }
  imageInterval_ = alignPeriod / double(S_);
  imuDiv_ = (alignPeriod * imuFreq) / I_;
  if (imuDiv_ < 1) {
    imuDiv_ = 1;
  } else if (imuDiv_ > 10) {
    imuDiv_ = 10;
  }
  filterWinSize_ = imuFreq / 20;
  if (filterWinSize_ < 20) {
    filterWinSize_ = 20;
  }
  goodExcitationThreshold_ = exciThr;
  filter_ = new GslGaussFilter(filterWinSize_,alpha);
  splineFile_.open("spline_1.csv",std::ios::out);
  splineFile_ << "[sample]:t,tx,ty,tz\n";
  splineFile_ << "[bspline]:t,px,py,pz,vx,vy,vz,ax,ay,az,anorm,accx,accy,accz,accnorm \n";
  splineFile_ << "[gaussFilter]:t,accx,accy,accz,afx,afy,afz\n";
  reset();
}

bool VOAlignedRealWorld::alignToRealWorld() {
  if (slidePose_.size() == S_) {
    std::map<double,Eigen::Vector3d> filteredAccel;
    //gauss filter for filtering accel noise
    if (accelGaussFiltering(filteredAccel)) {
      std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> alignedAccel;
      evalAccelByBSpline(filteredAccel,alignedAccel);
      calScaleAndR0(alignedAccel);
      reset();
    }
  }
  return alignedSuccessFlg_;
}


void VOAlignedRealWorld::update(const FramePtr& f,const MeasurementTimeline<IMU>& imuMeas) {
  //return if null pointer
  static double lastFrame = 0;
  if (f == nullptr || imuMeas.empty() || (f->timestamp_ - lastFrame) < imageInterval_) {
    return;
  }
  lastFrame = f->timestamp_;
  //get accel datas between frame at end of slidePose and current frame
  if (!slidePose_.empty()) {
    double curFrameTimestamp = f->timestamp_;
    double lastFrameTimestamp = slidePose_.rbegin()->first;
    std::map<double,IMU>::const_iterator it0 = imuMeas.getLowIter(lastFrameTimestamp);
    for (auto it = it0; it != imuMeas.measMap_.end(); it++) {
      double imuTimestamp = it->second.timestamp_;
      //skip imu datas that is newer than current frame
      if (imuTimestamp < lastFrameTimestamp) {
        continue;
      }
      //get out when imu id older than last pose
      if (imuTimestamp > curFrameTimestamp) {
        break;
      }
      slideAccel_[imuTimestamp] = it->second.acc_;
    }
  }
  //insert current frame pose
  Eigen::Isometry3d Tc0ck;
  Tc0ck.setIdentity();
  Tc0ck.rotate(f->ERwc());
  Tc0ck.pretranslate(f->EWtC());
  slidePose_[f->timestamp_] = Tc0ck;
  //slide old pose and accel
  slideWindow();
}

void VOAlignedRealWorld::slideWindow() {
  if (slidePose_.size() <= S_) {
    return;
  }
  //slide window of accel
  while (!slideAccel_.empty() && slideAccel_.begin()->first < slidePose_.begin()->first) {
    slideAccel_.erase(slideAccel_.begin());
  }
  //slide window of pose
  slidePose_.erase(slidePose_.begin());
}

bool VOAlignedRealWorld::accelGaussFiltering(std::map<double,Eigen::Vector3d>& filteredAccel) {
  if (slideAccel_.size() < (imuDiv_ * I_) / 2) {
    std::cout << "Slide Accel data is not enough!" << std::endl;
    return false;
  }
  filteredAccel.clear();
  //imu div
  std::vector<Eigen::Vector3d> rawDatas,filteredDatas;
  std::vector<double> filteredTimestamp;
  size_t cnt = 0;
  for (auto it = slideAccel_.begin(); it != slideAccel_.end(); it++) {
    if (cnt++ % imuDiv_ != 0) {
      continue;
    }
    double timestamp = it->first;
    rawDatas.push_back(it->second);
    filteredTimestamp.push_back(timestamp);
  }
  if (enableGaussFilter_) {
    filteredDatas = filter_->apply(rawDatas);
  } else {
    filteredDatas = rawDatas;
  }
  //calculate excitation
  Eigen::Vector3d accSum;
  accSum.setZero();
  for (size_t i = 0; i < filteredDatas.size(); i++) {
    double t = filteredTimestamp[i];
    filteredAccel[t] = filteredDatas[i];
    accSum += filteredDatas[i];
    splineFile_ << "[gaussFilter]:" << t << ","
                << slideAccel_[t].x() << ","
                << slideAccel_[t].y() << ","
                << slideAccel_[t].z() << ","
                << filteredAccel[t].x() << ","
                << filteredAccel[t].y() << ","
                << filteredAccel[t].z() << std::endl;
  }
  if (filteredDatas.size() > 10) {
    Eigen::Vector3d averAccel = accSum / filteredDatas.size();
    double sumErr = 0;
    for (auto s : filteredDatas) {
      sumErr += (s - averAccel).norm();
    }
    double exciate = sumErr / filteredDatas.size();
    splineFile_ << "[excitation]:" << exciate << std::endl;
    return exciate > goodExcitationThreshold_;
  }
  splineFile_ << "[excitation]:None!" << std::endl;
  return false;
}

bool VOAlignedRealWorld::checkAccelExcitation(std::map<double, Eigen::Vector3d>& filteredAccel) {
  if (slidePose_.size() != S_ || filteredAccel.empty()) {
    return false;
  }
  size_t goodExciatitionSize = 0;
  auto accIt = filteredAccel.begin();
  for (auto it = slidePose_.begin(); it != slidePose_.end(); it++) {
    Eigen::Vector3d accSum;
    accSum.setZero();
    std::vector<Eigen::Vector3d> periodAccel;
    double frameTimestamp = it->first;
    while (accIt != filteredAccel.end() && accIt->first < frameTimestamp) {
      accSum += accIt->second;
      periodAccel.push_back(accIt->second);
      accIt++;
    }
    if (periodAccel.size() > imuDiv_ / 2) {
      double sumErr = 0;
      Eigen::Vector3d averAccel = accSum / periodAccel.size();
      for (auto s : periodAccel) {
        sumErr += (s - averAccel).norm();
      }
      double exciate = sumErr / periodAccel.size();
      splineFile_ << "[excitation]:" << exciate << std::endl;
      if (exciate > goodExcitationThreshold_) {
        goodExciatitionSize++;
      }
    }
    //break if no accel datas
    if (accIt == filteredAccel.end()) {
      break;
    }
  }
  return goodExciatitionSize > S_ / 2;
}

void VOAlignedRealWorld::evalAccelByBSpline(std::map<double,Eigen::Vector3d>& filteredAccel,std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>>& alignedAccel) {
  alignedAccel.clear();
  if (slidePose_.size() != S_) {
    return;
  }
  Eigen::Matrix<double,S_,1> x;
  Eigen::Matrix<double,S_,D_> y;
  size_t cnt = 0;
  for (std::map<double,Eigen::Isometry3d>::const_iterator it = slidePose_.begin();it != slidePose_.end(); it++) {
    x(cnt,0) = it->first;
    y.row(cnt) = it->second.translation();
    cnt++;
    splineFile_ << "[sample]:" << it->first << "," << it->second.translation().x() << "," << it->second.translation().y() << "," << it->second.translation().z() << std::endl;
  }
  BSplineX<S_,D_,K_> splinex(x,y);
  std::vector<Eigen::Vector3d> splineAcc,imuAccel;
  double t0 = x(2,0);
  double t1 = x(S_ - 2,0);
  for (auto it = filteredAccel.begin(); it != filteredAccel.end(); it++) {
    double imuTimestamp = it->first;
    if (imuTimestamp < t0) {
      continue;
    }
    if (imuTimestamp > t1) {
      break;
    }
    Eigen::Vector3d g(0.,0.,9.81);
    Eigen::Matrix<double, 1, D_> pos,vel,acc;
    if (splinex.getSecondDifference(it->first, acc)) {
      splinex.getEvalValue(it->first,pos);
      splinex.getFirstDifference(it->first,vel);
      Eigen::Quaterniond q_slerp = getSlerpQuaternion(imuTimestamp);
      Eigen::Vector3d cAcc = q_slerp.toRotationMatrix() * Rcb_ * it->second;
      alignedAccel.push_back(std::make_pair(acc,cAcc));
      splineFile_ << "[bspline]:" << imuTimestamp << "," << pos.x() << "," << pos.y() << "," << pos.z() << ","
                  << vel.x() << "," << vel.y() << "," << vel.z() << ","
                  << acc.x() << "," << acc.y() << "," << acc.z() << "," << acc.norm() << ","
                  << it->second.x() << "," << it->second.y() << "," << it->second.z() << "," << (it->second + g).norm() << std::endl;
    }
  }
  splineFile_ << "===========================================================================" << std::endl;
}

void VOAlignedRealWorld::calScaleAndR0(std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>>& alignedAccel) {
  if (alignedAccel.size() < 10) {
    alignedSuccessFlg_ = false;
    return;
  }
  const int N = alignedAccel.size() * 3;
  Eigen::MatrixXd A{N,4};
  Eigen::VectorXd b{N};
  A.setZero();
  b.setZero();
  for (size_t i = 0; i < alignedAccel.size(); i++) {
    Eigen::Vector3d bsplineAcc = alignedAccel[i].first;
    Eigen::Vector3d camAcc = alignedAccel[i].second;
    A.block<3,1>(i * 3,0) = bsplineAcc;
    A.block<3,3>(i * 3,1) = Eigen::Matrix3d::Identity();
    b.block<3,1>(i * 3,0) = camAcc;
  }

  Eigen::VectorXd x;
  x = (A.transpose() * A).ldlt().solve(A.transpose() * b);
  std::cout << "[VOAlign] X = " << x.transpose() << std::endl;
}

Eigen::Quaterniond VOAlignedRealWorld::getSlerpQuaternion(double t) const {
  Eigen::Quaterniond q0,q1;
  double t0,t1;
  if (slidePose_.begin()->first > t || slidePose_.rbegin()->first < t) {
    std::cout << "[Slerp]:Failure!This can't be happened! input timestamp must be in the slide pose!" << std::endl;
    return Eigen::Quaterniond::Identity();
  }
  for (auto it = slidePose_.begin(); it != slidePose_.end(); it++) {
    if (it->first < t) {
      t0 = it->first;
      q0 = Eigen::Quaterniond(it->second.rotation());
    }
    if (it->first > t) {
      t1 = it->first;
      q1 = Eigen::Quaterniond(it->second.rotation());
      break;
    }
  }
  double skew = (t - t0) / (t1 - t0);
  return q0.slerp(skew,q1);
}