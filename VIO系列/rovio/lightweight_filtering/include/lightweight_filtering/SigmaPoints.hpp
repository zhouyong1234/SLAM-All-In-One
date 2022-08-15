/*
 * SigmaPoints.hpp
 *
 *  Created on: Feb 9, 2014
 *      Author: Bloeschm
 */

#ifndef LWF_SIGMAPOINTS_HPP_
#define LWF_SIGMAPOINTS_HPP_

#include "lightweight_filtering/State.hpp"
#include "lightweight_filtering/common.hpp"

namespace LWF{

template<typename State, unsigned int N, unsigned int L, unsigned int O>
class SigmaPoints{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef State mtState;
  static const unsigned int N_ = N;
  static const unsigned int L_ = L;
  static const unsigned int O_ = O;
  double wm_ = 1.0;
  double wc_ = 1.0;
  double wc0_ = 1.0;
  double gamma_ = 1.0;
  mtState sigmaPoints_[N];
  Eigen::MatrixXd S_;
  SigmaPoints(){
    static_assert(N_+O_<=L_, "Bad dimensions for sigmapoints");
    S_.setZero();
  };
  virtual ~SigmaPoints(){};
  void getMean(mtState& mean) const{
    typename mtState::mtDifVec vec;
    typename mtState::mtDifVec vecTemp;
    vec.setZero();
    for(unsigned int i=1;i<N_;i++){
      sigmaPoints_[i].boxMinus(sigmaPoints_[0],vecTemp);
      vec = vec + wm_*vecTemp;
    }
    sigmaPoints_[0].boxPlus(vec,mean);
  };
  void getCovarianceMatrix(Eigen::MatrixXd& C) const{
    mtState mean;
    getMean(mean);
    getCovarianceMatrix(mean,C);
  };
  void getCovarianceMatrix(const mtState& mean, Eigen::MatrixXd& C) const{
    typename mtState::mtDifVec vec;
    sigmaPoints_[0].boxMinus(mean,vec);
    Eigen::MatrixXd dynVec;
    dynVec = vec;
    C = dynVec*dynVec.transpose()*(wc0_+ wc_*(L_-N_));
    for(unsigned int i=1;i<N_;i++){
      sigmaPoints_[i].boxMinus(mean,vec);
      dynVec = vec;
      C += dynVec*dynVec.transpose()*wc_;
    }
  };
  template<typename State2, unsigned int N2, unsigned int O2>
  void getCovarianceMatrix(const SigmaPoints<State2,N2,L_,O2>& sigmaPoints2, Eigen::MatrixXd& C) const{
    mtState mean1;
    getMean(mean1);
    State2 mean2;
    sigmaPoints2.getMean(mean2);
    getCovarianceMatrix(sigmaPoints2,mean1,mean2,C);
  };
  template<typename State2, unsigned int N2, unsigned int O2>
  void getCovarianceMatrix(const SigmaPoints<State2,N2,L_,O2>& sigmaPoints2, const mtState& mean1, const State2& mean2, Eigen::MatrixXd& C) const{
    typename mtState::mtDifVec vec1;
    typename State2::mtDifVec vec2;
    Eigen::MatrixXd dynVec1;
    Eigen::MatrixXd dynVec2;
    (*this)(0).boxMinus(mean1,vec1);
    sigmaPoints2(0).boxMinus(mean2,vec2);
    dynVec1 = vec1;
    dynVec2 = vec2;
    C = dynVec1*dynVec2.transpose()*wc0_;
    for(unsigned int i=1;i<L_;i++){
      (*this)(i).boxMinus(mean1,vec1);
      sigmaPoints2(i).boxMinus(mean2,vec2);
      dynVec1 = vec1;
      dynVec2 = vec2;
      C += dynVec1*dynVec2.transpose()*wc_;
    }
  };
  template<typename State2>
  void extendZeroMeanGaussian(const SigmaPoints<State2,N_-2*mtState::D_,L_,O_>& sigmaPoints2, const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q){ // samples the last dimensions
    Eigen::MatrixXd C = Q.transpose()*sigmaPoints2.S_.inverse();
    Eigen::LDLT<Eigen::MatrixXd> ldltOfP(P-Q.transpose()*sigmaPoints2.S_.transpose().inverse()*sigmaPoints2.S_.inverse()*Q);
    Eigen::MatrixXd ldltL = ldltOfP.matrixL();
    Eigen::MatrixXd ldltD = ldltOfP.vectorD().asDiagonal();
    for(unsigned int i=0;i<mtState::D_;i++){
      if(ldltD(i,i)>0){
        ldltD(i,i) = std::sqrt(ldltD(i,i));
      } else if(ldltD(i,i)==0) {
        ldltD(i,i) = 0.0;
        std::cout << "CAUTION: Covariance matrix is only positive SEMIdefinite" << std::endl;
      } else {
        ldltD(i,i) = 0.0;
        std::cout << "ERROR: Covariance matrix is not positive semidefinite" << std::endl;
      }
    }
    if(ldltOfP.info()==Eigen::NumericalIssue) std::cout << "Numerical issues while computing Cholesky Matrix" << std::endl;
    S_ = ldltOfP.transpositionsP().transpose()*ldltL*ldltD;

    sigmaPoints_[0].setIdentity();
    int otherSize = (N_-2*mtState::D_-1)/2;
    for(unsigned int i=0;i<otherSize;i++){
      sigmaPoints_[0].boxPlus(C.col(i)*gamma_,sigmaPoints_[i+1]);
      sigmaPoints_[0].boxPlus(-C.col(i)*gamma_,sigmaPoints_[i+1+otherSize]);
    }
    for(unsigned int i=0;i<mtState::D_;i++){
      sigmaPoints_[0].boxPlus(S_.col(i)*gamma_,sigmaPoints_[2*otherSize+i+1]);
      sigmaPoints_[0].boxPlus(-S_.col(i)*gamma_,sigmaPoints_[2*otherSize+i+1+mtState::D_]);
    }
  };
  void computeFromGaussian(const mtState mean, const Eigen::MatrixXd &P){
    static_assert(N_==2*mtState::D_+1, "computeFromGaussian() requires matching sigmapoints size");
    Eigen::LDLT<Eigen::MatrixXd> ldltOfP(P);
    Eigen::MatrixXd ldltL = ldltOfP.matrixL();
    Eigen::MatrixXd ldltD = ldltOfP.vectorD().asDiagonal();
    for(unsigned int i=0;i<mtState::D_;i++){
      if(ldltD(i,i)>0){
        ldltD(i,i) = std::sqrt(ldltD(i,i));
      } else if(ldltD(i,i)==0) {
        ldltD(i,i) = 0.0;
        std::cout << "CAUTION: Covariance matrix is only positive SEMIdefinite" << std::endl;
      } else {
        ldltD(i,i) = 0.0;
        std::cout << "ERROR: Covariance matrix is not positive semidefinite" << std::endl;
      }
    }
    if(ldltOfP.info()==Eigen::NumericalIssue) std::cout << "Numerical issues while computing Cholesky Matrix" << std::endl;
    S_ = ldltOfP.transpositionsP().transpose()*ldltL*ldltD;

    sigmaPoints_[0] = mean;
    for(unsigned int i=0;i<mtState::D_;i++){
      mean.boxPlus(S_.col(i)*gamma_,sigmaPoints_[i+1]);
      mean.boxPlus(-S_.col(i)*gamma_,sigmaPoints_[i+1+mtState::D_]);
    }
  };
  void computeFromGaussian(const mtState mean, const Eigen::MatrixXd &P, const Eigen::MatrixXd &Q){
    static_assert(N_==2*mtState::D_+1, "computeFromGaussian() requires matching sigmapoints size");
    // The square root of a matrix is not unique, here is a way to influence it by means of an orthonormal matrix Q
    Eigen::LDLT<Eigen::MatrixXd> ldltOfP(Q.transpose()*P*Q); //
    Eigen::MatrixXd ldltL = ldltOfP.matrixL();
    Eigen::MatrixXd ldltD = ldltOfP.vectorD().asDiagonal();
    for(unsigned int i=0;i<mtState::D_;i++){
      if(ldltD(i,i)>0){
        ldltD(i,i) = std::sqrt(ldltD(i,i));
      } else if(ldltD(i,i)==0) {
        ldltD(i,i) = 0.0;
        std::cout << "CAUTION: Covariance matrix is only positive SEMIdefinite" << std::endl;
      } else {
        ldltD(i,i) = 0.0;
        std::cout << "ERROR: Covariance matrix is not positive semidefinite" << std::endl;
      }
    }
    if(ldltOfP.info()==Eigen::NumericalIssue) std::cout << "Numerical issues while computing Cholesky Matrix" << std::endl;
    S_ = Q*ldltOfP.transpositionsP().transpose()*ldltL*ldltD;

    sigmaPoints_[0] = mean;
    for(unsigned int i=0;i<mtState::D_;i++){
      mean.boxPlus(S_.col(i)*gamma_,sigmaPoints_[i+1]);
      mean.boxPlus(-S_.col(i)*gamma_,sigmaPoints_[i+1+mtState::D_]);
    }
  };
  void computeFromZeroMeanGaussian(const Eigen::MatrixXd &P){
    mtState identity;		// is initialized to 0 by default constructors
    identity.setIdentity();
    computeFromGaussian(identity,P);
  };
  const mtState& operator()(unsigned int i) const{
    assert(i<L_);
    if(i<O_){
      return sigmaPoints_[0];
    } else if(i<O_+N_){
      return sigmaPoints_[i-O_];
    } else {
      return sigmaPoints_[0];
    }
  };
  mtState& operator()(unsigned int i) {
    assert(i<L_);
    if(i<O_){
      return sigmaPoints_[0];
    } else if(i<O_+N_){
      return sigmaPoints_[i-O_];
    } else {
      return sigmaPoints_[0];
    }
  };
  void computeParameter(double alpha,double beta,double kappa){
    const unsigned int D = (L_-1)/2;
    const double lambda = alpha*alpha*(D+kappa)-D;
    gamma_ = sqrt(lambda + D);
    wm_ = 1/(2*(D+lambda));
    wc_ = wm_;
    wc0_ = lambda/(D+lambda)+(1-alpha*alpha+beta);
  };
};

}

#endif /* LWF_SIGMAPOINTS_HPP_ */
