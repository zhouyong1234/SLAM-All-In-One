/*
 * Prediction.hpp
 *
 *  Created on: Feb 9, 2014
 *      Author: Bloeschm
 */

#ifndef LWF_PREDICTIONMODEL_HPP_
#define LWF_PREDICTIONMODEL_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/ModelBase.hpp"
#include "lightweight_filtering/PropertyHandler.hpp"

namespace LWF{

template<typename FilterState>
class Prediction: public ModelBase<Prediction<FilterState>,typename FilterState::mtState,typename FilterState::mtState,typename FilterState::mtPredictionNoise>, public PropertyHandler{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef ModelBase<Prediction<FilterState>,typename FilterState::mtState,typename FilterState::mtState,typename FilterState::mtPredictionNoise> mtModelBase;
  typedef FilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
  typedef typename mtModelBase::mtInputTuple mtInputTuple;
  typedef typename mtFilterState::mtPredictionMeas mtMeas;
  typedef typename mtFilterState::mtPredictionNoise mtNoise;
  mtMeas meas_;
  Eigen::MatrixXd prenoiP_;
  Eigen::MatrixXd prenoiPinv_;
  bool disablePreAndPostProcessingWarning_;
  Prediction(): prenoiP_((int)(mtNoise::D_),(int)(mtNoise::D_)),
                prenoiPinv_((int)(mtNoise::D_),(int)(mtNoise::D_)){
    prenoiP_.setIdentity();
    prenoiP_ *= 0.0001;
    mtNoise n;
    n.setIdentity();
    n.registerCovarianceToPropertyHandler_(prenoiP_,this,"PredictionNoise.");
    disablePreAndPostProcessingWarning_ = false;
    refreshProperties();
  };
  virtual ~Prediction(){};
  void refreshProperties(){
    prenoiPinv_.setIdentity();
    prenoiP_.llt().solveInPlace(prenoiPinv_);
  }
  void eval_(mtState& x, const mtInputTuple& inputs, double dt) const{
    evalPrediction(x,std::get<0>(inputs),std::get<1>(inputs),dt);
  }
  template<int i,typename std::enable_if<i==0>::type* = nullptr>
  void jacInput_(Eigen::MatrixXd& F, const mtInputTuple& inputs, double dt) const{
    jacPreviousState(F,std::get<0>(inputs),dt);
  }
  template<int i,typename std::enable_if<i==1>::type* = nullptr>
  void jacInput_(Eigen::MatrixXd& F, const mtInputTuple& inputs, double dt) const{
    jacNoise(F,std::get<0>(inputs),dt);
  }
  virtual void evalPrediction(mtState& x, const mtState& previousState, const mtNoise& noise, double dt) const = 0;
  virtual void evalPredictionShort(mtState& x, const mtState& previousState, double dt) const{
    mtNoise n; // TODO get static for Identity()
    n.setIdentity();
    evalPrediction(x,previousState,n,dt);
  }
  virtual void jacPreviousState(Eigen::MatrixXd& F, const mtState& previousState, double dt) const = 0;
  virtual void jacNoise(Eigen::MatrixXd& F, const mtState& previousState, double dt) const = 0;
  virtual void noMeasCase(mtFilterState& filterState, mtMeas& meas, double dt){};
  virtual void preProcess(mtFilterState& filterState, const mtMeas& meas, double dt){
    if(!disablePreAndPostProcessingWarning_){
      std::cout << "Warning: prediction preProcessing is not implemented!" << std::endl;
    }
  };
  virtual void postProcess(mtFilterState& filterState, const mtMeas& meas, double dt){
    if(!disablePreAndPostProcessingWarning_){
      std::cout << "Warning: prediction postProcessing is not implemented!" << std::endl;
    }
  };
  int performPrediction(mtFilterState& filterState, const mtMeas& meas, double dt){
    switch(filterState.mode_){
      case ModeEKF:
        return performPredictionEKF(filterState,meas,dt);
      case ModeUKF:
        return performPredictionUKF(filterState,meas,dt);
      case ModeIEKF:
        return performPredictionEKF(filterState,meas,dt);
      default:
        return performPredictionEKF(filterState,meas,dt);
    }
  }
  int performPrediction(mtFilterState& filterState, double dt){
    mtMeas meas;
    meas.setIdentity();
    noMeasCase(filterState,meas,dt);
    return performPrediction(filterState,meas,dt);
  }
  int performPredictionEKF(mtFilterState& filterState, const mtMeas& meas, double dt){
    preProcess(filterState,meas,dt);
    meas_ = meas;
    this->jacPreviousState(filterState.F_,filterState.state_,dt);
    this->jacNoise(filterState.G_,filterState.state_,dt);
    this->evalPredictionShort(filterState.state_,filterState.state_,dt);
    filterState.cov_ = filterState.F_*filterState.cov_*filterState.F_.transpose() + filterState.G_*prenoiP_*filterState.G_.transpose();
    filterState.state_.fix();
    enforceSymmetry(filterState.cov_);
    filterState.t_ += dt;
    postProcess(filterState,meas,dt);
    return 0;
  }
  int performPredictionUKF(mtFilterState& filterState, const mtMeas& meas, double dt){
    filterState.refreshNoiseSigmaPoints(prenoiP_);
    preProcess(filterState,meas,dt);
    meas_ = meas;
    filterState.stateSigmaPoints_.computeFromGaussian(filterState.state_,filterState.cov_);

    // Prediction
    for(unsigned int i=0;i<filterState.stateSigmaPoints_.L_;i++){
      this->evalPrediction(filterState.stateSigmaPointsPre_(i),filterState.stateSigmaPoints_(i),filterState.stateSigmaPointsNoi_(i),dt);
    }
    // Calculate mean and variance
    filterState.stateSigmaPointsPre_.getMean(filterState.state_);
    filterState.stateSigmaPointsPre_.getCovarianceMatrix(filterState.state_,filterState.cov_);
    filterState.state_.fix();
    filterState.t_ += dt;
    postProcess(filterState,meas,dt);
    return 0;
  }
  int predictMerged(mtFilterState& filterState, double tTarget, const std::map<double,mtMeas>& measMap){
    switch(filterState.mode_){
      case ModeEKF:
        return predictMergedEKF(filterState,tTarget,measMap);
      case ModeUKF:
        return predictMergedUKF(filterState,tTarget,measMap);
      case ModeIEKF:
        return predictMergedEKF(filterState,tTarget,measMap);
      default:
        return predictMergedEKF(filterState,tTarget,measMap);
    }
  }
  virtual int predictMergedEKF(mtFilterState& filterState, const double tTarget, const std::map<double,mtMeas>& measMap){
    const typename std::map<double,mtMeas>::const_iterator itMeasStart = measMap.upper_bound(filterState.t_);
    if(itMeasStart == measMap.end()) return 0;
    typename std::map<double,mtMeas>::const_iterator itMeasEnd = measMap.lower_bound(tTarget);
    if(itMeasEnd != measMap.end()) ++itMeasEnd;
    double dT = std::min(std::prev(itMeasEnd)->first,tTarget)-filterState.t_;
    if(dT <= 0) return 0;

    // Compute mean Measurement
    mtMeas meanMeas;
    typename mtMeas::mtDifVec vec;
    typename mtMeas::mtDifVec difVec;
    vec.setZero();
    double t = itMeasStart->first;
    for(typename std::map<double,mtMeas>::const_iterator itMeas=next(itMeasStart);itMeas!=itMeasEnd;itMeas++){
      itMeas->second.boxMinus(itMeasStart->second,difVec);
      vec = vec + difVec*(std::min(itMeas->first,tTarget)-t);
      t = std::min(itMeas->first,tTarget);
    }
    vec = vec/dT;
    itMeasStart->second.boxPlus(vec,meanMeas);

    preProcess(filterState,meanMeas,dT);
    meas_ = meanMeas;
    this->jacPreviousState(filterState.F_,filterState.state_,dT);
    this->jacNoise(filterState.G_,filterState.state_,dT); // Works for time continuous parametrization of noise
    for(typename std::map<double,mtMeas>::const_iterator itMeas=itMeasStart;itMeas!=itMeasEnd;itMeas++){
      meas_ = itMeas->second;
      this->evalPredictionShort(filterState.state_,filterState.state_,std::min(itMeas->first,tTarget)-filterState.t_);
      filterState.t_ = std::min(itMeas->first,tTarget);
    }
    filterState.cov_ = filterState.F_*filterState.cov_*filterState.F_.transpose() + filterState.G_*prenoiP_*filterState.G_.transpose();
    filterState.state_.fix();
    enforceSymmetry(filterState.cov_);
    filterState.t_ = std::min(std::prev(itMeasEnd)->first,tTarget);
    postProcess(filterState,meanMeas,dT);
    return 0;
  }
  virtual int predictMergedUKF(mtFilterState& filterState, double tTarget, const std::map<double,mtMeas>& measMap){
    filterState.refreshNoiseSigmaPoints(prenoiP_);
    const typename std::map<double,mtMeas>::const_iterator itMeasStart = measMap.upper_bound(filterState.t_);
    if(itMeasStart == measMap.end()) return 0;
    const typename std::map<double,mtMeas>::const_iterator itMeasEnd = measMap.upper_bound(tTarget);
    if(itMeasEnd == measMap.begin()) return 0;
    double dT = std::prev(itMeasEnd)->first-filterState.t_;

    // Compute mean Measurement
    mtMeas meanMeas;
    typename mtMeas::mtDifVec vec;
    typename mtMeas::mtDifVec difVec;
    vec.setZero();
    double t = itMeasStart->first;
    for(typename std::map<double,mtMeas>::const_iterator itMeas=next(itMeasStart);itMeas!=itMeasEnd;itMeas++){
      itMeasStart->second.boxMinus(itMeas->second,difVec);
      vec = vec + difVec*(itMeas->first-t);
      t = itMeas->first;
    }
    vec = vec/dT;
    itMeasStart->second.boxPlus(vec,meanMeas);

    preProcess(filterState,meanMeas,dT);
    meas_ = meanMeas;
    filterState.stateSigmaPoints_.computeFromGaussian(filterState.state_,filterState.cov_);

    // Prediction
    for(unsigned int i=0;i<filterState.stateSigmaPoints_.L_;i++){
      this->evalPrediction(filterState.stateSigmaPointsPre_(i),filterState.stateSigmaPoints_(i),filterState.stateSigmaPointsNoi_(i),dT);
    }
    filterState.stateSigmaPointsPre_.getMean(filterState.state_);
    filterState.stateSigmaPointsPre_.getCovarianceMatrix(filterState.state_,filterState.cov_);
    filterState.state_.fix();
    filterState.t_ = std::prev(itMeasEnd)->first;
    postProcess(filterState,meanMeas,dT);
    return 0;
  }
  bool testPredictionJacs(double d = 1e-6,double th = 1e-6,double dt = 0.1){
    mtState state;
    mtMeas meas;
    unsigned int s = 1;
    state.setRandom(s);
    meas.setRandom(s);
    return testPredictionJacs(state,meas,d,th,dt);
  }
  bool testPredictionJacs(const mtState& state, const mtMeas& meas, double d = 1e-6,double th = 1e-6,double dt = 0.1){
    mtInputTuple inputs;
    std::get<0>(inputs) = state;
    std::get<1>(inputs).setIdentity(); // Noise is always set to zero for Jacobians
    meas_ = meas;
    return this->testJacs(inputs,d,th,dt);
  }
};

}

#endif /* LWF_PREDICTIONMODEL_HPP_ */
