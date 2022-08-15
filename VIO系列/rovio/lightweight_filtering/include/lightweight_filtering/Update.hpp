/*
 * Update.hpp
 *
 *  Created on: Feb 9, 2014
 *      Author: Bloeschm
 */

#ifndef LWF_UPDATEMODEL_HPP_
#define LWF_UPDATEMODEL_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/ModelBase.hpp"
#include "lightweight_filtering/PropertyHandler.hpp"
#include "lightweight_filtering/SigmaPoints.hpp"
#include "lightweight_filtering/OutlierDetection.hpp"
#include <list>
#include <Eigen/StdVector>

namespace LWF{

template<typename Innovation, typename FilterState, typename Meas, typename Noise, typename OutlierDetection = OutlierDetectionDefault, bool isCoupled = false>
class Update: public ModelBase<Update<Innovation,FilterState,Meas,Noise,OutlierDetection,isCoupled>,Innovation,typename FilterState::mtState,Noise>, public PropertyHandler{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static_assert(!isCoupled || Noise::D_ == FilterState::noiseExtensionDim_,"Noise Size for coupled Update must match noise extension of prediction!");
  typedef ModelBase<Update<Innovation,FilterState,Meas,Noise,OutlierDetection,isCoupled>,Innovation,typename FilterState::mtState,Noise> mtModelBase;
  typedef FilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
  typedef typename mtModelBase::mtInputTuple mtInputTuple;
  typedef typename mtFilterState::mtPredictionMeas mtPredictionMeas;
  typedef typename mtFilterState::mtPredictionNoise mtPredictionNoise;
  typedef Innovation mtInnovation;
  typedef Meas mtMeas;
  typedef Noise mtNoise;
  typedef OutlierDetection mtOutlierDetection;
  mtMeas meas_; // TODO change to pointer, or remove
  static const bool coupledToPrediction_ = isCoupled;
  bool useSpecialLinearizationPoint_;
  bool useImprovedJacobian_;
  bool hasConverged_;
  bool successfulUpdate_;
  mutable bool cancelIteration_;
  mutable int candidateCounter_;
  mutable Eigen::MatrixXd H_;
  Eigen::MatrixXd Hlin_;
  Eigen::MatrixXd boxMinusJac_;
  Eigen::MatrixXd Hn_;
  Eigen::MatrixXd updnoiP_;
  Eigen::MatrixXd noiP_;
  Eigen::MatrixXd preupdnoiP_;
  Eigen::MatrixXd C_;
  mtInnovation y_;
  mutable Eigen::MatrixXd Py_;
  Eigen::MatrixXd Pyinv_;
  typename mtInnovation::mtDifVec innVector_;
  mtInnovation yIdentity_;
  typename mtState::mtDifVec updateVec_;
  mtState linState_;
  double updateVecNorm_;
  Eigen::MatrixXd K_;
  Eigen::MatrixXd Pyx_;
  mutable typename mtState::mtDifVec difVecLinInv_;

  SigmaPoints<mtState,2*mtState::D_+1,2*(mtState::D_+mtNoise::D_)+1,0> stateSigmaPoints_;
  SigmaPoints<mtNoise,2*mtNoise::D_+1,2*(mtState::D_+mtNoise::D_)+1,2*mtState::D_> stateSigmaPointsNoi_;
  SigmaPoints<mtInnovation,2*(mtState::D_+mtNoise::D_)+1,2*(mtState::D_+mtNoise::D_)+1,0> innSigmaPoints_;
  SigmaPoints<mtNoise,2*(mtNoise::D_+mtPredictionNoise::D_)+1,2*(mtState::D_+mtNoise::D_+mtPredictionNoise::D_)+1,2*(mtState::D_)> coupledStateSigmaPointsNoi_;
  SigmaPoints<mtInnovation,2*(mtState::D_+mtNoise::D_+mtPredictionNoise::D_)+1,2*(mtState::D_+mtNoise::D_+mtPredictionNoise::D_)+1,0> coupledInnSigmaPoints_;
  SigmaPoints<LWF::VectorElement<mtState::D_>,2*mtState::D_+1,2*mtState::D_+1,0> updateVecSP_;
  SigmaPoints<mtState,2*mtState::D_+1,2*mtState::D_+1,0> posterior_;
  double alpha_;
  double beta_;
  double kappa_;
  double updateVecNormTermination_;
  int maxNumIteration_;
  int iterationNum_;
  mtOutlierDetection outlierDetection_;
  unsigned int numSequences;
  bool disablePreAndPostProcessingWarning_;
  Update(): H_((int)(mtInnovation::D_),(int)(mtState::D_)),
      Hlin_((int)(mtInnovation::D_),(int)(mtState::D_)),
      boxMinusJac_((int)(mtState::D_),(int)(mtState::D_)),
      Hn_((int)(mtInnovation::D_),(int)(mtNoise::D_)),
      updnoiP_((int)(mtNoise::D_),(int)(mtNoise::D_)),
      noiP_((int)(mtNoise::D_),(int)(mtNoise::D_)),
      preupdnoiP_((int)(mtPredictionNoise::D_),(int)(mtNoise::D_)),
      C_((int)(mtState::D_),(int)(mtInnovation::D_)),
      Py_((int)(mtInnovation::D_),(int)(mtInnovation::D_)),
      Pyinv_((int)(mtInnovation::D_),(int)(mtInnovation::D_)),
      K_((int)(mtState::D_),(int)(mtInnovation::D_)),
      Pyx_((int)(mtInnovation::D_),(int)(mtState::D_)){
    alpha_ = 1e-3;
    beta_ = 2.0;
    kappa_ = 0.0;
    updateVecNormTermination_ = 1e-6;
    maxNumIteration_  = 10;
    updnoiP_.setIdentity();
    updnoiP_ *= 0.0001;
    noiP_.setZero();
    preupdnoiP_.setZero();
    useSpecialLinearizationPoint_ = false;
    useImprovedJacobian_ = false;
    yIdentity_.setIdentity();
    updateVec_.setIdentity();
    refreshNoiseSigmaPoints();
    refreshUKFParameter();
    mtNoise n;
    n.setIdentity();
    n.registerCovarianceToPropertyHandler_(updnoiP_,this,"UpdateNoise.");
    doubleRegister_.registerScalar("alpha",alpha_);
    doubleRegister_.registerScalar("beta",beta_);
    doubleRegister_.registerScalar("kappa",kappa_);
    doubleRegister_.registerScalar("updateVecNormTermination",updateVecNormTermination_);
    intRegister_.registerScalar("maxNumIteration",maxNumIteration_);
    outlierDetection_.setEnabledAll(false);
    numSequences = 1;
    disablePreAndPostProcessingWarning_ = false;
  };
  virtual ~Update(){};
  void refreshNoiseSigmaPoints(){
    if(noiP_ != updnoiP_){
      noiP_ = updnoiP_;
      stateSigmaPointsNoi_.computeFromZeroMeanGaussian(noiP_);
    }
  }
  void refreshUKFParameter(){
    stateSigmaPoints_.computeParameter(alpha_,beta_,kappa_);
    innSigmaPoints_.computeParameter(alpha_,beta_,kappa_);
    coupledInnSigmaPoints_.computeParameter(alpha_,beta_,kappa_);
    updateVecSP_.computeParameter(alpha_,beta_,kappa_);
    posterior_.computeParameter(alpha_,beta_,kappa_);
    stateSigmaPointsNoi_.computeParameter(alpha_,beta_,kappa_);
    stateSigmaPointsNoi_.computeFromZeroMeanGaussian(noiP_);
    coupledStateSigmaPointsNoi_.computeParameter(alpha_,beta_,kappa_);
  }
  void refreshProperties(){
    refreshPropertiesCustom();
    refreshUKFParameter();
  }
  virtual void refreshPropertiesCustom(){}
  void eval_(mtInnovation& x, const mtInputTuple& inputs, double dt) const{
    evalInnovation(x,std::get<0>(inputs),std::get<1>(inputs));
  }
  template<int i,typename std::enable_if<i==0>::type* = nullptr>
  void jacInput_(Eigen::MatrixXd& F, const mtInputTuple& inputs, double dt) const{
    jacState(F,std::get<0>(inputs));
  }
  template<int i,typename std::enable_if<i==1>::type* = nullptr>
  void jacInput_(Eigen::MatrixXd& F, const mtInputTuple& inputs, double dt) const{
    jacNoise(F,std::get<0>(inputs));
  }
  virtual void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const = 0;
  virtual void evalInnovationShort(mtInnovation& y, const mtState& state) const{
    mtNoise n; // TODO get static for Identity()
    n.setIdentity();
    evalInnovation(y,state,n);
  }
  virtual void jacState(Eigen::MatrixXd& F, const mtState& state) const = 0;
  virtual void jacNoise(Eigen::MatrixXd& F, const mtState& state) const = 0;
  virtual void preProcess(mtFilterState& filterState, const mtMeas& meas, bool& isFinished){
    isFinished = false;
    if(!disablePreAndPostProcessingWarning_){
      std::cout << "Warning: update preProcessing is not implemented!" << std::endl;
    }
  }
  virtual bool extraOutlierCheck(const mtState& state) const{
    return hasConverged_;
  }
  virtual bool generateCandidates(const mtFilterState& filterState, mtState& candidate) const{
    candidate = filterState.state_;
    candidateCounter_++;
    if(candidateCounter_<=1)
      return true;
    else
      return false;
  }
  virtual void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    isFinished = true;
    if(!disablePreAndPostProcessingWarning_){
      std::cout << "Warning: update postProcessing is not implemented!" << std::endl;
    }
  }
  int performUpdate(mtFilterState& filterState, const mtMeas& meas){
    bool isFinished = true;
    int r = 0;
    do {
      preProcess(filterState,meas,isFinished);
      if(!isFinished){
        switch(filterState.mode_){
          case ModeEKF:
            r = performUpdateEKF(filterState,meas);
            break;
          case ModeUKF:
            r = performUpdateUKF(filterState,meas);
            break;
          case ModeIEKF:
            r = performUpdateIEKF(filterState,meas);
            break;
          default:
            r = performUpdateEKF(filterState,meas);
            break;
        }
      }
      postProcess(filterState,meas,outlierDetection_,isFinished);
      filterState.state_.fix();
      enforceSymmetry(filterState.cov_);
    } while (!isFinished);
    return r;
  }
  int performUpdateEKF(mtFilterState& filterState, const mtMeas& meas){
    meas_ = meas;
    if(!useSpecialLinearizationPoint_){
      this->jacState(H_,filterState.state_);
      Hlin_ = H_;
      this->jacNoise(Hn_,filterState.state_);
      this->evalInnovationShort(y_,filterState.state_);
    } else {
      filterState.state_.boxPlus(filterState.difVecLin_,linState_);
      this->jacState(H_,linState_);
      if(useImprovedJacobian_){
        filterState.state_.boxMinusJac(linState_,boxMinusJac_);
        Hlin_ = H_*boxMinusJac_;
      } else {
        Hlin_ = H_;
      }
      this->jacNoise(Hn_,linState_);
      this->evalInnovationShort(y_,linState_);
    }

    if(isCoupled){
      C_ = filterState.G_*preupdnoiP_*Hn_.transpose();
      Py_ = Hlin_*filterState.cov_*Hlin_.transpose() + Hn_*updnoiP_*Hn_.transpose() + Hlin_*C_ + C_.transpose()*Hlin_.transpose();
    } else {
      Py_ = Hlin_*filterState.cov_*Hlin_.transpose() + Hn_*updnoiP_*Hn_.transpose();
    }
    y_.boxMinus(yIdentity_,innVector_);

    // Outlier detection // TODO: adapt for special linearization point
    outlierDetection_.doOutlierDetection(innVector_,Py_,Hlin_);
    Pyinv_.setIdentity();
    Py_.llt().solveInPlace(Pyinv_);

    // Kalman Update
    if(isCoupled){
      K_ = (filterState.cov_*Hlin_.transpose()+C_)*Pyinv_;
    } else {
      K_ = filterState.cov_*Hlin_.transpose()*Pyinv_;
    }
    filterState.cov_ = filterState.cov_ - K_*Py_*K_.transpose();
    if(!useSpecialLinearizationPoint_){
      updateVec_ = -K_*innVector_;
    } else {
      filterState.state_.boxMinus(linState_,difVecLinInv_);
      updateVec_ = -K_*(innVector_+H_*difVecLinInv_); // includes correction for offseted linearization point, dif must be recomputed (a-b != (-(b-a)))
    }
    filterState.state_.boxPlus(updateVec_,filterState.state_);
    return 0;
  }
  int performUpdateIEKF(mtFilterState& filterState, const mtMeas& meas){
    meas_ = meas;
    successfulUpdate_ = false;
    candidateCounter_ = 0;

    std::vector<double> scores;
    std::vector<mtState, Eigen::aligned_allocator<mtState>> states;
    double bestScore = -1.0;
    mtState bestState;
    MXD bestCov;

    while(generateCandidates(filterState,linState_)){
      cancelIteration_ = false;
      hasConverged_ = false;
      for(iterationNum_=0;iterationNum_<maxNumIteration_ && !hasConverged_ && !cancelIteration_;iterationNum_++){
        this->jacState(H_,linState_);
        this->jacNoise(Hn_,linState_);
        this->evalInnovationShort(y_,linState_);

        if(isCoupled){
          C_ = filterState.G_*preupdnoiP_*Hn_.transpose();
          Py_ = H_*filterState.cov_*H_.transpose() + Hn_*updnoiP_*Hn_.transpose() + H_*C_ + C_.transpose()*H_.transpose();
        } else {
          Py_ = H_*filterState.cov_*H_.transpose() + Hn_*updnoiP_*Hn_.transpose();
        }
        y_.boxMinus(yIdentity_,innVector_);

        // Outlier detection
        outlierDetection_.doOutlierDetection(innVector_,Py_,H_);
        Pyinv_.setIdentity();
        Py_.llt().solveInPlace(Pyinv_);

        // Kalman Update
        if(isCoupled){
          K_ = (filterState.cov_*H_.transpose()+C_)*Pyinv_;
        } else {
          K_ = filterState.cov_*H_.transpose()*Pyinv_;
        }
        filterState.state_.boxMinus(linState_,difVecLinInv_);
        updateVec_ = -K_*(innVector_+H_*difVecLinInv_)+difVecLinInv_; // includes correction for offseted linearization point, dif must be recomputed (a-b != (-(b-a)))
        linState_.boxPlus(updateVec_,linState_);
        updateVecNorm_ = updateVec_.norm();
        hasConverged_ = updateVecNorm_<=updateVecNormTermination_;
      }
      if(extraOutlierCheck(linState_)){
        successfulUpdate_ = true;
        double score = (innVector_.transpose()*Pyinv_*innVector_)(0);
        scores.push_back(score);
        states.push_back(linState_);
        if(bestScore == -1.0 || score < bestScore){
          bestScore = score;
          bestState = linState_;
          bestCov = filterState.cov_ - K_*Py_*K_.transpose();
        }
      }
    }

    if(successfulUpdate_){
      if(scores.size() == 1){
        filterState.state_ = bestState;
        filterState.cov_ = bestCov;
      } else {
        bool foundOtherMin = false;
        for(auto it = states.begin();it!=states.end();it++){
          bestState.boxMinus(*it,difVecLinInv_);
          if(difVecLinInv_.norm()>2*updateVecNormTermination_){
            foundOtherMin = true;
            break;
          }
        }
        if(!foundOtherMin){
          filterState.state_ = bestState;
          filterState.cov_ = bestCov;
        } else {
          successfulUpdate_ = false;
        }
      }
    }
    return 0;
  }
  int performUpdateUKF(mtFilterState& filterState, const mtMeas& meas){
    meas_ = meas;
    handleUpdateSigmaPoints<isCoupled>(filterState);
    y_.boxMinus(yIdentity_,innVector_);

    outlierDetection_.doOutlierDetection(innVector_,Py_,Pyx_);
    Pyinv_.setIdentity();
    Py_.llt().solveInPlace(Pyinv_);

    // Kalman Update
    K_ = Pyx_.transpose()*Pyinv_;
    filterState.cov_ = filterState.cov_ - K_*Py_*K_.transpose();
    updateVec_ = -K_*innVector_;

    // Adapt for proper linearization point
    updateVecSP_.computeFromZeroMeanGaussian(filterState.cov_);
    for(unsigned int i=0;i<2*mtState::D_+1;i++){
      filterState.state_.boxPlus(updateVec_+updateVecSP_(i).v_,posterior_(i));
    }
    posterior_.getMean(filterState.state_);
    posterior_.getCovarianceMatrix(filterState.state_,filterState.cov_);
    return 0;
  }
  template<bool IC = isCoupled, typename std::enable_if<(IC)>::type* = nullptr>
  void handleUpdateSigmaPoints(mtFilterState& filterState){
    coupledStateSigmaPointsNoi_.extendZeroMeanGaussian(filterState.stateSigmaPointsNoi_,updnoiP_,preupdnoiP_);
    for(unsigned int i=0;i<coupledInnSigmaPoints_.L_;i++){
      this->evalInnovation(coupledInnSigmaPoints_(i),filterState.stateSigmaPointsPre_(i),coupledStateSigmaPointsNoi_(i));
    }
    coupledInnSigmaPoints_.getMean(y_);
    coupledInnSigmaPoints_.getCovarianceMatrix(y_,Py_);
    coupledInnSigmaPoints_.getCovarianceMatrix(filterState.stateSigmaPointsPre_,Pyx_);
  }
  template<bool IC = isCoupled, typename std::enable_if<(!IC)>::type* = nullptr>
  void handleUpdateSigmaPoints(mtFilterState& filterState){
    refreshNoiseSigmaPoints();
    stateSigmaPoints_.computeFromGaussian(filterState.state_,filterState.cov_);
    for(unsigned int i=0;i<innSigmaPoints_.L_;i++){
      this->evalInnovation(innSigmaPoints_(i),stateSigmaPoints_(i),stateSigmaPointsNoi_(i));
    }
    innSigmaPoints_.getMean(y_);
    innSigmaPoints_.getCovarianceMatrix(y_,Py_);
    innSigmaPoints_.getCovarianceMatrix(stateSigmaPoints_,Pyx_);
  }
  bool testUpdateJacs(double d = 1e-6,double th = 1e-6){
    mtState state;
    mtMeas meas;
    unsigned int s = 1;
    state.setRandom(s);
    meas.setRandom(s);
    return testUpdateJacs(state,meas,d,th);
  }
  bool testUpdateJacs(const mtState& state, const mtMeas& meas, double d = 1e-6,double th = 1e-6){
    mtInputTuple inputs;
    const double dt = 1.0;
    std::get<0>(inputs) = state;
    std::get<1>(inputs).setIdentity(); // Noise is always set to zero for Jacobians
    meas_ = meas;
    return this->testJacs(inputs,d,th,dt);
  }
};

}

#endif /* LWF_UPDATEMODEL_HPP_ */
