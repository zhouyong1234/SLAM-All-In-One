//
// Created by kdq on 2021/6/22.
//
#pragma once
#include <map>
#include "BSplineX.hpp"
#include "Frame.hpp"
#include "MeasurementTimeline.hpp"
#include "GslGaussFilter.hpp"
using namespace vio;
class VOAlignedRealWorld {
 public:

  /** \brief Constructor
   * @param imuFreq --- imu frequency
   * @param imageFreq --- image frequency
   * @param alignPeriod --- period of alignments
   * @param alpha --- gauss filter parameter
   * @param exciThr --- excitation threshold default value 0.2 at reference documents
   */
  VOAlignedRealWorld(Eigen::Matrix3d Rcb,int imuFreq,int imageFreq,double alignPeriod,double alpha,double exciThr = 0.2,bool enableGaussFilter = true);

  /** \brief Destructor
   *
   */
  ~VOAlignedRealWorld() {
    delete filter_;
  }

  /** \brief  update slide accel and frame
 * @param f
 * @param imuMeas
 */
  void update(const FramePtr& f,const MeasurementTimeline<IMU>& imuMeas);


  /** \brief align vo with real world through calculating scale and Rwc0
   * @param f  --- frame pointer
   * @param imuMeas --- imu measurements
   * @return flag of aligned success
   */
  bool alignToRealWorld();

  /** \brief check this frame is not included in slide pose
   * @param timestamp
   * @return
   */
  bool frameAlreadyInsert(double timestamp) const {
    return slidePose_.count(timestamp);
  }
  /** \brief reset system for next alignment
   */
  void reset() {
    slidePose_.clear();
    slideAccel_.clear();
    obviousExcitationSize_ = 0;
    alignedSuccessFlg_ = false;
  }

 private:
  const static int D_ = 3;
  const static int S_ = 15;
  const static int K_ = 5;
  const static int I_ = S_ * 20;
  static_assert(S_ > 5);
  int imageDiv_;
  double imageInterval_;
  int imuDiv_;
  int filterWinSize_;
  std::map<double,Eigen::Vector3d> slideAccel_;
  std::map<double,Eigen::Isometry3d> slidePose_;
  GslGaussFilter* filter_;
  int obviousExcitationSize_;
  bool alignedSuccessFlg_;
  double goodExcitationThreshold_;
  bool enableGaussFilter_;
  std::ofstream splineFile_;
  Eigen::Matrix3d Rcb_;
  /** \brief accel apply gauss filtering
   *
   */
  bool accelGaussFiltering(std::map<double,Eigen::Vector3d>& filteredAccel);

  /** \brief check accelerator whether has enough excitation or not between two frames
   * @param filteredAccel --- accel data for filtering
   * @return
   */
  bool checkAccelExcitation(std::map<double,Eigen::Vector3d>& filteredAccel);

  /** \brief eval accel by b-splinex
   */
  void evalAccelByBSpline(std::map<double,Eigen::Vector3d>& filteredAccel,std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>>& alignedAccel);

  /** \brief calculate scale and Rwc0 from vo to real world
   *
   */
  void calScaleAndR0(std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>>& alignedAccel);

  /** \brief erase datas before timestamp t
   * @param t --- timestamp
   */
  void slideWindow();

  /** \brief get slerp quaternion from slide pose
   * @param t --- slerp timestamp
   * @return slerp quaternion
   */
  Eigen::Quaterniond getSlerpQuaternion(double t) const;

};
