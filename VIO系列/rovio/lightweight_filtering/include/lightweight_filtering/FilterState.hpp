/*
 * FilterState.hpp
 *
 *  Created on: Feb 9, 2014
 *      Author: Bloeschm
 */

#ifndef LWF_FILTERSTATE_HPP_
#define LWF_FILTERSTATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/SigmaPoints.hpp"

namespace LWF{

template<typename State, typename PredictionMeas, typename PredictionNoise, unsigned int noiseExtensionDim = 0>
class FilterState{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef State mtState;
  typedef PredictionMeas mtPredictionMeas;
  typedef PredictionNoise mtPredictionNoise;
  FilteringMode mode_;
  bool usePredictionMerge_;
  static constexpr unsigned int noiseExtensionDim_ = noiseExtensionDim;
  double t_;
  mtState state_;
  Eigen::MatrixXd cov_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd G_;
  SigmaPoints<mtState,2*mtState::D_+1,2*(mtState::D_+mtPredictionNoise::D_+noiseExtensionDim)+1,0> stateSigmaPoints_;
  SigmaPoints<mtPredictionNoise,2*mtPredictionNoise::D_+1,2*(mtState::D_+mtPredictionNoise::D_+noiseExtensionDim)+1,2*mtState::D_> stateSigmaPointsNoi_;
  SigmaPoints<mtState,2*(mtState::D_+mtPredictionNoise::D_)+1,2*(mtState::D_+mtPredictionNoise::D_+noiseExtensionDim)+1,0> stateSigmaPointsPre_;
  Eigen::MatrixXd prenoiP_; // automatic change tracking
  typename mtState::mtDifVec difVecLin_;
  double alpha_;
  double beta_;
  double kappa_;
  FilterState():  cov_((int)(mtState::D_),(int)(mtState::D_)),
                  F_((int)(mtState::D_),(int)(mtState::D_)),
                  G_((int)(mtState::D_),(int)(mtPredictionNoise::D_)),
                  prenoiP_((int)(mtPredictionNoise::D_),(int)(mtPredictionNoise::D_)){
    alpha_ = 1e-3;
    beta_ = 2.0;
    kappa_ = 0.0;
    mode_ = ModeEKF;
    usePredictionMerge_ = false;
    t_ = 0.0;
    state_.setIdentity();
    cov_.setIdentity();
    F_.setIdentity();
    G_.setZero();
    prenoiP_.setIdentity();
    prenoiP_ *= 0.0001;
    difVecLin_.setIdentity();
    refreshUKFParameter();
  }
  virtual ~FilterState(){};
  void refreshUKFParameter(){
    stateSigmaPoints_.computeParameter(alpha_,beta_,kappa_);
    stateSigmaPointsNoi_.computeParameter(alpha_,beta_,kappa_);
    stateSigmaPointsPre_.computeParameter(alpha_,beta_,kappa_);
    stateSigmaPointsNoi_.computeFromZeroMeanGaussian(prenoiP_);
  }
  void refreshNoiseSigmaPoints(const Eigen::MatrixXd& prenoiP){
    if(prenoiP_ != prenoiP){
      prenoiP_ = prenoiP;
      stateSigmaPointsNoi_.computeFromZeroMeanGaussian(prenoiP_);
    }
  }
};

}

#endif /* LWF_FILTERSTATE_HPP_ */
