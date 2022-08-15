/*
 * FilterBase.hpp
 *
 *  Created on: Feb 9, 2014
 *      Author: Bloeschm
 */

#ifndef LWF_FilterBase_HPP_
#define LWF_FilterBase_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/PropertyHandler.hpp"

namespace LWF{

template<typename Meas>
class MeasurementTimeline{
 public:
  typedef Meas mtMeas;
  std::map<double,mtMeas> measMap_;
  typename std::map<double,mtMeas>::iterator itMeas_;
  double maxWaitTime_;
  double minWaitTime_;
  MeasurementTimeline(){
    maxWaitTime_ = 0.1;
    minWaitTime_ = 0.0;
  };
  virtual ~MeasurementTimeline(){};
  void addMeas(const mtMeas& meas, const double& t){
    measMap_[t] = meas;
  }
  void clear()
  {
    measMap_.clear();
  }
  void clean(double t){
    while(measMap_.size() > 1 && measMap_.begin()->first<=t){
      measMap_.erase(measMap_.begin());
    }
  }
  bool getNextTime(double actualTime, double& nextTime){
    itMeas_ = measMap_.upper_bound(actualTime);
    if(itMeas_!=measMap_.end()){
      nextTime = itMeas_->first;
      return true;
    } else {
      return false;
    }
  }
  void waitTime(double actualTime, double& time){
    double measurementTime = actualTime-maxWaitTime_;
    if(!measMap_.empty() && measMap_.rbegin()->first + minWaitTime_ > measurementTime){
      measurementTime = measMap_.rbegin()->first + minWaitTime_;
    }
    if(time > measurementTime){
      time = measurementTime;
    }
  }
  bool getLastTime(double& lastTime){
    if(!measMap_.empty()){
      lastTime = measMap_.rbegin()->first;
      return true;
    } else {
      return false;
    }
  }
  bool hasMeasurementAt(double t){
    return measMap_.count(t)>0;
  }
};

template<typename Prediction,typename... Updates>
class FilterBase: public PropertyHandler{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Prediction mtPrediction;
  typedef typename mtPrediction::mtState mtState;
  static const unsigned int D_ = mtState::D_;
  static const int nUpdates_ = sizeof...(Updates);
  typedef typename mtPrediction::mtFilterState mtFilterState;
  mtFilterState safe_;
  mtFilterState front_;
  mtFilterState init_;
  MeasurementTimeline<typename mtPrediction::mtMeas> predictionTimeline_;
  std::tuple<MeasurementTimeline<typename Updates::mtMeas>...> updateTimelineTuple_;
  mtPrediction mPrediction_;
  typedef std::tuple<Updates...> mtUpdates;
  mtUpdates mUpdates_;
  double safeWarningTime_;
  double frontWarningTime_;
  bool gotFrontWarning_;
  bool updateToUpdateMeasOnly_;
  unsigned int logCountMerPre_;
  unsigned int logCountRegPre_;
  unsigned int logCountBadPre_;
  unsigned int logCountComUpd_;
  unsigned int logCountRegUpd_;
  bool logCountDiagnostics_;
  FilterBase(){
    init_.state_.setIdentity();
    init_.cov_.setIdentity();
    mPrediction_.doubleRegister_.registerScalar("alpha",init_.alpha_);
    mPrediction_.doubleRegister_.registerScalar("beta",init_.beta_);
    mPrediction_.doubleRegister_.registerScalar("kappa",init_.kappa_);
    init_.state_.registerElementsToPropertyHandler(this,"Init.State.");
    init_.state_.registerCovarianceToPropertyHandler_(init_.cov_,this,"Init.Covariance.");
    registerSubHandler("Prediction",mPrediction_);
    registerUpdates();
    reset();
    logCountDiagnostics_ = false;
    updateToUpdateMeasOnly_ = false;
  };
  virtual ~FilterBase(){
  };
  void reset(double t = 0.0){
    init_.t_ = t;
    init_.state_.fix();
    safe_ = init_;
    front_ = init_;
    safeWarningTime_ = t;
    frontWarningTime_ = t;
    gotFrontWarning_ = false;
  }
  template<int i=0, typename std::enable_if<(i<nUpdates_)>::type* = nullptr>
  void registerUpdates(){
    registerSubHandler("Update" + std::to_string(i),std::get<i>(mUpdates_));
    std::get<i>(mUpdates_).outlierDetection_.registerToPropertyHandler(&std::get<i>(mUpdates_),"MahalanobisTh");
    registerUpdates<i+1>();
  }
  template<int i=0, typename std::enable_if<(i>=nUpdates_)>::type* = nullptr>
  void registerUpdates(){
  }
  void addPredictionMeas(const typename Prediction::mtMeas& meas, double t){
    if(t<= safeWarningTime_) {
      std::cout << "[FilterBase::addPredictionMeas] Warning: included measurements at time " << t << " before safeTime " << safeWarningTime_ << std::endl;
    }

    if(t<= frontWarningTime_) gotFrontWarning_ = true;
    predictionTimeline_.addMeas(meas,t);
  }
  template<int i>
  void addUpdateMeas(const typename std::tuple_element<i,decltype(mUpdates_)>::type::mtMeas& meas, double t){
    if(t<= safeWarningTime_) {
      std::cout << "[FilterBase::addUpdateMeas] Warning: included measurements at time " << t << " before safeTime " << safeWarningTime_ << std::endl;
    }
    if(t<= frontWarningTime_) gotFrontWarning_ = true;
    std::get<i>(updateTimelineTuple_).addMeas(meas,t);
  }
  bool getSafeTime(double& safeTime){
    double maxPredictionTime;
    if(!predictionTimeline_.getLastTime(maxPredictionTime)){
      safeTime = safe_.t_;
      return false;
    }
    safeTime = maxPredictionTime;
    // Check if we have to wait for update measurements
    checkUpdateWaitTime(maxPredictionTime,safeTime);
    if(safeTime <= safe_.t_){
      safeTime = safe_.t_;
      return false;
    }
    return true;
  }
  template<int i=0, typename std::enable_if<(i<nUpdates_)>::type* = nullptr>
  void checkUpdateWaitTime(double actualTime,double& time){
    std::get<i>(updateTimelineTuple_).waitTime(actualTime,time);
    checkUpdateWaitTime<i+1>(actualTime,time);
  }
  template<int i=0, typename std::enable_if<(i>=nUpdates_)>::type* = nullptr>
  void checkUpdateWaitTime(double actualTime,double& time){
  }
  void updateSafe(const double* maxTime = nullptr){
    double nextSafeTime;
    bool gotSafeTime = getSafeTime(nextSafeTime);
    if(!gotSafeTime || (maxTime != nullptr && *maxTime < safe_.t_)){
      if(logCountDiagnostics_){
        std::cout << "Performed safe Update with RegPre: 0, MerPre: 0, BadPre: 0, RegUpd: 0, ComUpd: 0" << std::endl;
      }
      return;
    }
    if(maxTime != nullptr && nextSafeTime > *maxTime) nextSafeTime = *maxTime;
    if(front_.t_<=nextSafeTime && !gotFrontWarning_ && front_.t_>safe_.t_){
      safe_ = front_;
    }
    update(safe_,nextSafeTime);
    clean(safe_.t_);
    safeWarningTime_ = safe_.t_;
    if(logCountDiagnostics_){
      std::cout << "Performed safe Update with RegPre: " << logCountRegPre_ << ", MerPre: " << logCountMerPre_ << ", BadPre: " << logCountBadPre_ << ", RegUpd: " << logCountRegUpd_ << ", ComUpd: " << logCountComUpd_ << std::endl;
    }
  }
  void updateFront(const double& tEnd){
    updateSafe();
    if(gotFrontWarning_ || front_.t_<=safe_.t_){
      front_ = safe_;
    }
    update(front_,tEnd);
    frontWarningTime_ = front_.t_;
    gotFrontWarning_ = false;
  }
  void update(mtFilterState& filterState,const double& tEnd){
    double tNext = filterState.t_;
    logCountMerPre_ = 0;
    logCountRegPre_ = 0;
    logCountBadPre_ = 0;
    logCountComUpd_ = 0;
    logCountRegUpd_ = 0;
    while(filterState.t_<tEnd){
      tNext = tEnd;
      if(!getNextUpdate(filterState.t_,tNext) && updateToUpdateMeasOnly_){
        break; // Don't go further if there is no update available
      }
      int r = 0;
      if(filterState.usePredictionMerge_){
        r = mPrediction_.predictMerged(filterState,tNext,predictionTimeline_.measMap_);
        if(r!=0) std::cout << "Error during predictMerged: " << r << std::endl;
        logCountMerPre_++;
      } else {
        while(filterState.t_ < tNext && (predictionTimeline_.itMeas_ = predictionTimeline_.measMap_.upper_bound(filterState.t_)) != predictionTimeline_.measMap_.end()){
          r = mPrediction_.performPrediction(filterState,predictionTimeline_.itMeas_->second,std::min(predictionTimeline_.itMeas_->first,tNext)-filterState.t_);
          if(r!=0) std::cout << "Error during performPrediction: " << r << std::endl;
          logCountRegPre_++;
        }
      }
      if(filterState.t_ < tNext){
        r = mPrediction_.performPrediction(filterState,tNext-filterState.t_);
        if(r!=0) std::cout << "Error during performPrediction: " << r << std::endl;
        logCountBadPre_++;
      }
      doAvailableUpdates(filterState,tNext);
    }
  }
  template<int i=0, typename std::enable_if<(i<nUpdates_)>::type* = nullptr>
  bool getNextUpdate(double actualTime, double& nextTime){
    double tNextUpdate;
    bool gotMatchingUpdate = false;
    if(std::get<i>(updateTimelineTuple_).getNextTime(actualTime,tNextUpdate) && tNextUpdate < nextTime){
      gotMatchingUpdate = true;
      nextTime = tNextUpdate;
    }
    return gotMatchingUpdate | getNextUpdate<i+1>(actualTime, nextTime);
  }
  template<int i=0, typename std::enable_if<(i>=nUpdates_)>::type* = nullptr>
  bool getNextUpdate(double actualTime, double& nextTime){
    return false;
  }
  template<int i=0, typename std::enable_if<(i<nUpdates_)>::type* = nullptr>
  void doAvailableUpdates(mtFilterState& filterState, double tNext){
    if(std::get<i>(updateTimelineTuple_).hasMeasurementAt(tNext)){
          int r = std::get<i>(mUpdates_).performUpdate(filterState,std::get<i>(updateTimelineTuple_).measMap_[tNext]);
          if(r!=0) std::cout << "Error during update: " << r << std::endl;
          logCountRegUpd_++;
    }
    doAvailableUpdates<i+1>(filterState,tNext);
  }
  template<int i=0, typename std::enable_if<(i>=nUpdates_)>::type* = nullptr>
  void doAvailableUpdates(mtFilterState& filterState, double tNext){
  }
  void clean(const double& t){
    predictionTimeline_.clean(t);
    cleanUpdateTimeline(t);
  }
  template<int i=0, typename std::enable_if<(i<nUpdates_)>::type* = nullptr>
  void cleanUpdateTimeline(const double& t){
    std::get<i>(updateTimelineTuple_).clean(t);
    cleanUpdateTimeline<i+1>(t);
  }
  template<int i=0, typename std::enable_if<(i>=nUpdates_)>::type* = nullptr>
  void cleanUpdateTimeline(const double& t){
  }
};

}

#endif /* LWF_FilterBase_HPP_ */
