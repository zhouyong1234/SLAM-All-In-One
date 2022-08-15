#include "lightweight_filtering/TestClasses.hpp"
#include "lightweight_filtering/common.hpp"
#include "gtest/gtest.h"
#include <assert.h>

using namespace LWFTest;

typedef ::testing::Types<
    NonlinearTest,
    LinearTest
> TestClasses;

// The fixture for testing class PredictionModel
template<typename TestClass>
class PredictionModelTest : public ::testing::Test, public TestClass {
 protected:
  PredictionModelTest() {
    this->init(this->testState_,this->testUpdateMeas_,this->testPredictionMeas_);
    this->measMap_[0.1] = this->testPredictionMeas_;
    this->measMap_[0.2] = this->testPredictionMeas_;
    this->measMap_[0.4] = this->testPredictionMeas_;
  }
  virtual ~PredictionModelTest() {
  }
  using typename TestClass::mtState;
  using typename TestClass::mtFilterState;
  using typename TestClass::mtUpdateMeas;
  using typename TestClass::mtUpdateNoise;
  using typename TestClass::mtInnovation;
  using typename TestClass::mtPredictionNoise;
  using typename TestClass::mtPredictionMeas;
  using typename TestClass::mtUpdateExample;
  using typename TestClass::mtPredictionExample;
  using typename TestClass::mtPredictAndUpdateExample;
  mtPredictionExample testPrediction_;
  mtState testState_;
  mtPredictionMeas testPredictionMeas_;
  mtUpdateMeas testUpdateMeas_;
  const double dt_ = 0.1;
  std::map<double,mtPredictionMeas> measMap_;
};

TYPED_TEST_CASE(PredictionModelTest, TestClasses);

// Test constructors
TYPED_TEST(PredictionModelTest, constructors) {
  typename TestFixture::mtPredictionExample testPrediction;
  ASSERT_EQ((testPrediction.prenoiP_-Eigen::Matrix<double,TestFixture::mtState::D_,TestFixture::mtState::D_>::Identity()*0.0001).norm(),0.0);
}

// Test finite difference Jacobians
TYPED_TEST(PredictionModelTest, FDjacobians) {
  Eigen::MatrixXd F((int)(TestFixture::mtPredictionExample::mtState::D_),(int)(TestFixture::mtPredictionExample::mtState::D_));
  Eigen::MatrixXd F_FD((int)(TestFixture::mtPredictionExample::mtState::D_),(int)(TestFixture::mtPredictionExample::mtState::D_));
  typename TestFixture::mtPredictionNoise n;
  this->testPrediction_.meas_ = this->testPredictionMeas_;
  this->testPrediction_.template jacInputFD<0>(F_FD,std::forward_as_tuple(this->testState_,n),this->dt_,0.0000001);
  this->testPrediction_.template jacInput<0>(F,std::forward_as_tuple(this->testState_,n),this->dt_);
  Eigen::MatrixXd Fn((int)(TestFixture::mtPredictionExample::mtState::D_),(int)(TestFixture::mtPredictionExample::mtNoise::D_));
  Eigen::MatrixXd Fn_FD((int)(TestFixture::mtPredictionExample::mtState::D_),(int)(TestFixture::mtPredictionExample::mtNoise::D_));
  this->testPrediction_.template jacInputFD<1>(Fn_FD,std::forward_as_tuple(this->testState_,n),this->dt_,0.0000001);
  this->testPrediction_.template jacInput<1>(Fn,std::forward_as_tuple(this->testState_,n),this->dt_);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR((F-F_FD).norm(),0.0,1e-5);
      ASSERT_NEAR((Fn-Fn_FD).norm(),0.0,1e-5);
      break;
    case 1:
      ASSERT_NEAR((F-F_FD).norm(),0.0,1e-8);
      ASSERT_NEAR((Fn-Fn_FD).norm(),0.0,1e-8);
      break;
    default:
      ASSERT_NEAR((F-F_FD).norm(),0.0,1e-5);
      ASSERT_NEAR((Fn-Fn_FD).norm(),0.0,1e-5);
  };
}

// Test performPredictionEKF
TYPED_TEST(PredictionModelTest, performPredictionEKF) {
  typename TestFixture::mtPredictionExample::mtFilterState filterState;
  filterState.cov_.setIdentity();
  Eigen::MatrixXd F((int)(TestFixture::mtPredictionExample::mtState::D_),(int)(TestFixture::mtPredictionExample::mtState::D_));
  this->testPrediction_.meas_ = this->testPredictionMeas_;
  this->testPrediction_.jacPreviousState(F,this->testState_,this->dt_);
  Eigen::MatrixXd Fn((int)(TestFixture::mtPredictionExample::mtState::D_),(int)(TestFixture::mtPredictionExample::mtNoise::D_));
  this->testPrediction_.jacNoise(Fn,this->testState_,this->dt_);
  Eigen::MatrixXd predictedCov = F*filterState.cov_*F.transpose() + Fn*this->testPrediction_.prenoiP_*Fn.transpose();
  typename TestFixture::mtPredictionExample::mtState state;
  filterState.state_ = this->testState_;
  this->testPrediction_.performPredictionEKF(filterState,this->testPredictionMeas_,this->dt_);
  typename TestFixture::mtPredictionExample::mtState::mtDifVec dif;
  typename TestFixture::mtPredictionExample::mtState evalState;
  this->testPrediction_.evalPredictionShort(evalState,this->testState_,this->dt_);
  filterState.state_.boxMinus(evalState,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState.cov_-predictedCov).norm(),0.0,1e-6);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState.cov_-predictedCov).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState.cov_-predictedCov).norm(),0.0,1e-6);
  };
}

// Test comparePredict
TYPED_TEST(PredictionModelTest, comparePredict) {
  typename TestFixture::mtPredictionExample::mtFilterState filterState1;
  typename TestFixture::mtPredictionExample::mtFilterState filterState2;
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.000001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->testPrediction_.performPredictionEKF(filterState1,this->testPredictionMeas_,this->dt_);
  this->testPrediction_.performPredictionUKF(filterState2,this->testPredictionMeas_,this->dt_);
  typename TestFixture::mtPredictionExample::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-5);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-5);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-9);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-5);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-6);
  };
}

// Test predictMergedEKF
TYPED_TEST(PredictionModelTest, predictMergedEKF) {
  typename TestFixture::mtPredictionExample::mtFilterState filterState1;
  typename TestFixture::mtPredictionExample::mtFilterState filterState2;
  filterState1.cov_.setIdentity();
  double t = 0;
  double dt = this->measMap_.rbegin()->first-t;

  typename TestFixture::mtPredictionExample::mtMeas meanMeas;
  typename TestFixture::mtPredictionExample::mtMeas::mtDifVec vec;
  typename TestFixture::mtPredictionExample::mtMeas::mtDifVec difVec;
  vec.setZero();
  for(typename std::map<double,typename TestFixture::mtPredictionExample::mtMeas>::iterator it = next(this->measMap_.begin());it != this->measMap_.end();it++){
    this->measMap_.begin()->second.boxMinus(it->second,difVec);
    vec = vec + difVec;
  }
  vec = vec/this->measMap_.size();
  this->measMap_.begin()->second.boxPlus(vec,meanMeas);

  Eigen::MatrixXd F((int)(TestFixture::mtPredictionExample::mtState::D_),(int)(TestFixture::mtPredictionExample::mtState::D_));
  this->testPrediction_.meas_ = meanMeas;
  this->testPrediction_.jacPreviousState(F,this->testState_,dt);
  Eigen::MatrixXd Fn((int)(TestFixture::mtPredictionExample::mtState::D_),(int)(TestFixture::mtPredictionExample::mtNoise::D_));
  this->testPrediction_.jacNoise(Fn,this->testState_,dt);
  Eigen::MatrixXd predictedCov = F*filterState1.cov_*F.transpose() + Fn*this->testPrediction_.prenoiP_*Fn.transpose();
  filterState1.state_ = this->testState_;
  this->testPrediction_.predictMergedEKF(filterState1,this->measMap_.rbegin()->first,this->measMap_);
  filterState2.state_ = this->testState_;
  for(typename std::map<double,typename TestFixture::mtPredictionExample::mtMeas>::iterator it = this->measMap_.begin();it != this->measMap_.end();it++){
    this->testPrediction_.meas_ = it->second;
    this->testPrediction_.evalPredictionShort(filterState2.state_,filterState2.state_,it->first-t);
    t = it->first;
  }
  typename TestFixture::mtPredictionExample::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState1.cov_-predictedCov).norm(),0.0,1e-6);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState1.cov_-predictedCov).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState1.cov_-predictedCov).norm(),0.0,1e-6);
  };
}

// Test predictMergedUKF
TYPED_TEST(PredictionModelTest, predictMergedUKF) {
  typename TestFixture::mtPredictionExample::mtFilterState filterState1;
  typename TestFixture::mtPredictionExample::mtFilterState filterState2;
  filterState1.cov_.setIdentity();
  filterState2.cov_.setIdentity();
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  double t = 0;
  double dt = this->measMap_.rbegin()->first-t;

  typename TestFixture::mtPredictionExample::mtMeas meanMeas;
  typename TestFixture::mtPredictionExample::mtMeas::mtDifVec vec;
  typename TestFixture::mtPredictionExample::mtMeas::mtDifVec difVec;
  vec.setZero();
  for(typename std::map<double,typename TestFixture::mtPredictionExample::mtMeas>::iterator it = next(this->measMap_.begin());it != this->measMap_.end();it++){
    this->measMap_.begin()->second.boxMinus(it->second,difVec);
    vec = vec + difVec;
  }
  vec = vec/this->measMap_.size();
  this->measMap_.begin()->second.boxPlus(vec,meanMeas);

  filterState1.stateSigmaPoints_.computeFromGaussian(filterState1.state_,filterState1.cov_);
  for(unsigned int i=0;i<filterState1.stateSigmaPoints_.L_;i++){
    this->testPrediction_.meas_ = meanMeas;
    this->testPrediction_.evalPrediction(filterState1.stateSigmaPointsPre_(i),filterState1.stateSigmaPoints_(i),filterState1.stateSigmaPointsNoi_(i),dt);
  }
  filterState1.stateSigmaPointsPre_.getMean(filterState1.state_);
  filterState1.stateSigmaPointsPre_.getCovarianceMatrix(filterState1.state_,filterState1.cov_);
  this->testPrediction_.predictMergedUKF(filterState2,this->measMap_.rbegin()->first,this->measMap_);
  typename TestFixture::mtPredictionExample::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState2.cov_-filterState1.cov_).norm(),0.0,1e-6);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState2.cov_-filterState1.cov_).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState2.cov_-filterState1.cov_).norm(),0.0,1e-6);
  };
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
