#include "lightweight_filtering/TestClasses.hpp"
#include "lightweight_filtering/common.hpp"
#include "gtest/gtest.h"
#include <assert.h>

using namespace LWFTest;

typedef ::testing::Types<
    NonlinearTest,
    LinearTest
> TestClasses;

// The fixture for testing class UpdateModel
template<typename TestClass>
class UpdateModelTest : public ::testing::Test, public TestClass {
 public:
  UpdateModelTest() {
    this->init(this->testState_,this->testUpdateMeas_,this->testPredictionMeas_);
  }
  virtual ~UpdateModelTest() {
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
  using typename TestClass::mtOutlierDetectionExample;
  mtUpdateExample testUpdate_;
  mtPredictAndUpdateExample testPredictAndUpdate_;
  mtPredictionExample testPrediction_;
  mtState testState_;
  mtUpdateMeas testUpdateMeas_;
  mtPredictionMeas testPredictionMeas_;
  const double dt_ = 0.1;
};

TYPED_TEST_CASE(UpdateModelTest, TestClasses);

// Test constructors
TYPED_TEST(UpdateModelTest, constructors) {
  typename TestFixture::mtUpdateExample testUpdate;
  bool coupledToPrediction = testUpdate.coupledToPrediction_;
  ASSERT_EQ(coupledToPrediction,false);
  ASSERT_EQ((testUpdate.updnoiP_-Eigen::Matrix<double,TestFixture::mtUpdateNoise::D_,TestFixture::mtUpdateNoise::D_>::Identity()*0.0001).norm(),0.0);
  typename TestFixture::mtUpdateExample::mtNoise::mtDifVec dif;
  typename TestFixture::mtUpdateExample::mtNoise noise;
  noise.setIdentity();
  typename TestFixture::mtUpdateExample::mtNoise mean;
  testUpdate.stateSigmaPointsNoi_.getMean(mean);
  mean.boxMinus(noise,dif);
  ASSERT_NEAR(dif.norm(),0.0,1e-6);
  Eigen::MatrixXd C((int)(TestFixture::mtUpdateExample::mtNoise::D_),(int)(TestFixture::mtUpdateExample::mtNoise::D_));
  testUpdate.stateSigmaPointsNoi_.getCovarianceMatrix(C);
  ASSERT_NEAR((testUpdate.updnoiP_-C).norm(),0.0,1e-8);
  typename TestFixture::mtPredictAndUpdateExample testPredictAndUpdate;
  coupledToPrediction = testPredictAndUpdate.coupledToPrediction_;
  ASSERT_EQ(coupledToPrediction,true);
  ASSERT_EQ((testPredictAndUpdate.updnoiP_-Eigen::Matrix<double,TestFixture::mtPredictAndUpdateExample::mtNoise::D_,TestFixture::mtPredictAndUpdateExample::mtNoise::D_>::Identity()*0.0001).norm(),0.0);
}

// Test finite difference Jacobians
TYPED_TEST(UpdateModelTest, FDjacobians) {
  typename TestFixture::mtUpdateExample::mtNoise n;
  n.setIdentity();
  this->testUpdate_.meas_ = this->testUpdateMeas_;
  Eigen::MatrixXd F((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtState::D_));
  Eigen::MatrixXd F_FD((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtState::D_));
  this->testUpdate_.template jacInputFD<0>(F_FD,std::forward_as_tuple(this->testState_,n),this->dt_,0.0000001);
  this->testUpdate_.template jacInput<0>(F,std::forward_as_tuple(this->testState_,n));
  Eigen::MatrixXd H((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtNoise::D_));
  Eigen::MatrixXd H_FD((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtNoise::D_));
  this->testUpdate_.template jacInputFD<1>(H_FD,std::forward_as_tuple(this->testState_,n),this->dt_,0.0000001);
  this->testUpdate_.template jacInput<1>(H,std::forward_as_tuple(this->testState_,n));
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR((F-F_FD).norm(),0.0,1e-5);
      ASSERT_NEAR((H-H_FD).norm(),0.0,1e-5);
      break;
    case 1:
      ASSERT_NEAR((F-F_FD).norm(),0.0,1e-8);
      ASSERT_NEAR((H-H_FD).norm(),0.0,1e-8);
      break;
    default:
      ASSERT_NEAR((F-F_FD).norm(),0.0,1e-5);
      ASSERT_NEAR((H-H_FD).norm(),0.0,1e-5);
  }
}

// Test performUpdateEKF
TYPED_TEST(UpdateModelTest, performUpdateEKF) {
  this->testUpdate_.meas_ = this->testUpdateMeas_;
  typename TestFixture::mtUpdateExample::mtFilterState filterState;
  filterState.cov_.setIdentity();
  Eigen::MatrixXd H((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtState::D_));
  this->testUpdate_.jacState(H,this->testState_);
  Eigen::MatrixXd Hn((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtNoise::D_));
  this->testUpdate_.jacNoise(Hn,this->testState_);

  typename TestFixture::mtUpdateExample::mtInnovation y;
  this->testUpdate_.evalInnovationShort(y,this->testState_);
  typename TestFixture::mtUpdateExample::mtInnovation yIdentity;
  yIdentity.setIdentity();
  typename TestFixture::mtUpdateExample::mtInnovation::mtDifVec innVector;

  typename TestFixture::mtUpdateExample::mtState stateUpdated;
  filterState.state_ = this->testState_;

  // Update
  MXD Py = H*filterState.cov_*H.transpose() + Hn*this->testUpdate_.updnoiP_*Hn.transpose();
  y.boxMinus(yIdentity,innVector);
  MXD Pyinv = Py.inverse();

  // Kalman Update
  Eigen::Matrix<double,TestFixture::mtUpdateExample::mtState::D_,TestFixture::mtUpdateExample::mtInnovation::D_> K = filterState.cov_*H.transpose()*Pyinv;
  MXD updateCov = filterState.cov_ - K*Py*K.transpose();
  typename TestFixture::mtUpdateExample::mtState::mtDifVec updateVec;
  updateVec = -K*innVector;
  filterState.state_.boxPlus(updateVec,stateUpdated);

  this->testUpdate_.performUpdateEKF(filterState,this->testUpdateMeas_);
  typename TestFixture::mtUpdateExample::mtState::mtDifVec dif;
  filterState.state_.boxMinus(stateUpdated,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-6);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-6);
  }
}

// Test updateEKFWithOutlier
TYPED_TEST(UpdateModelTest, updateEKFWithOutlier) {
  this->testUpdate_.meas_ = this->testUpdateMeas_;
  typename TestFixture::mtUpdateExample::mtFilterState filterState;
  filterState.cov_.setIdentity();
  Eigen::MatrixXd H((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtState::D_));
  this->testUpdate_.jacState(H,this->testState_);
  Eigen::MatrixXd Hn((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtNoise::D_));
  this->testUpdate_.jacNoise(Hn,this->testState_);

  typename TestFixture::mtUpdateExample::mtInnovation y;
  this->testUpdate_.evalInnovationShort(y,this->testState_);
  typename TestFixture::mtUpdateExample::mtInnovation yIdentity;
  yIdentity.setIdentity();
  typename TestFixture::mtUpdateExample::mtInnovation::mtDifVec innVector;

  typename TestFixture::mtUpdateExample::mtState stateUpdated;
  filterState.state_ = this->testState_;

  // Update
  MXD Py = H*filterState.cov_*H.transpose() + Hn*this->testUpdate_.updnoiP_*Hn.transpose();
  y.boxMinus(yIdentity,innVector);
  Py.block(0,0,TestFixture::mtUpdateExample::mtInnovation::D_,3).setZero();
  Py.block(0,0,3,TestFixture::mtUpdateExample::mtInnovation::D_).setZero();
  Py.block(0,0,3,3).setIdentity();
  H.block(0,0,3,TestFixture::mtUpdateExample::mtState::D_).setZero();
  MXD Pyinv = Py.inverse();

  // Kalman Update
  Eigen::Matrix<double,TestFixture::mtUpdateExample::mtState::D_,TestFixture::mtUpdateExample::mtInnovation::D_> K = filterState.cov_*H.transpose()*Pyinv;
  MXD updateCov = filterState.cov_ - K*Py*K.transpose();
  typename TestFixture::mtUpdateExample::mtState::mtDifVec updateVec;
  updateVec = -K*innVector;
  filterState.state_.boxPlus(updateVec,stateUpdated);

  this->testUpdate_.outlierDetection_.setEnabledAll(true);
  this->testUpdate_.performUpdateEKF(filterState,this->testUpdateMeas_);
  typename TestFixture::mtUpdateExample::mtState::mtDifVec dif;
  filterState.state_.boxMinus(stateUpdated,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-6);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-6);
  }
}

// Test compareUpdate
TYPED_TEST(UpdateModelTest, compareUpdate) {
  typename TestFixture::mtUpdateExample::mtFilterState filterState1;
  typename TestFixture::mtUpdateExample::mtFilterState filterState2;
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.000001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->testUpdate_.performUpdateEKF(filterState1,this->testUpdateMeas_);
  this->testUpdate_.performUpdateUKF(filterState2,this->testUpdateMeas_);
  typename TestFixture::mtUpdateExample::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-5);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-5);
  }

  // Test with outlier detection
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.000001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->testUpdate_.outlierDetection_.setEnabledAll(true);
  this->testUpdate_.performUpdateEKF(filterState1,this->testUpdateMeas_);
  this->testUpdate_.performUpdateUKF(filterState2,this->testUpdateMeas_);
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-5);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-5);
  }
}

// Test performPredictionAndUpdateEKF
TYPED_TEST(UpdateModelTest, performPredictionAndUpdateEKF) { // Only tested without cross-correlation
  typename TestFixture::mtUpdateExample::mtFilterState filterState1;
  typename TestFixture::mtUpdateExample::mtFilterState filterState2;
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.000001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->testPrediction_.performPredictionEKF(filterState1,this->testPredictionMeas_,this->dt_);
  this->testUpdate_.performUpdateEKF(filterState1,this->testUpdateMeas_);
  this->testPrediction_.performPredictionEKF(filterState2,this->testPredictionMeas_,this->dt_);
  this->testPredictAndUpdate_.performUpdateEKF(filterState2,this->testUpdateMeas_);
  typename TestFixture::mtUpdateExample::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-8);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-8);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-8);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-8);
  }

  // With outlier
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.000001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->testUpdate_.outlierDetection_.setEnabledAll(true);
  this->testPredictAndUpdate_.outlierDetection_.setEnabledAll(true);
  this->testPrediction_.performPredictionEKF(filterState1,this->testPredictionMeas_,this->dt_);
  this->testUpdate_.performUpdateEKF(filterState1,this->testUpdateMeas_);
  this->testPrediction_.performPredictionEKF(filterState2,this->testPredictionMeas_,this->dt_);
  this->testPredictAndUpdate_.performUpdateEKF(filterState2,this->testUpdateMeas_);
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-8);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-8);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-8);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-8);
  }
}

// Test performPredictionAndUpdateUKF
TYPED_TEST(UpdateModelTest, performPredictionAndUpdateUKF) {
  typename TestFixture::mtUpdateExample::mtFilterState filterState1;
  typename TestFixture::mtUpdateExample::mtFilterState filterState2;
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.000001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->testPrediction_.performPredictionUKF(filterState1,this->testPredictionMeas_,this->dt_);
  this->testUpdate_.performUpdateUKF(filterState1,this->testUpdateMeas_);
  this->testPrediction_.performPredictionUKF(filterState2,this->testPredictionMeas_,this->dt_);
  this->testPredictAndUpdate_.performUpdateUKF(filterState2,this->testUpdateMeas_);
  typename TestFixture::mtUpdateExample::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-4); // Increased difference comes because of different sigma points
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-6);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,2e-10);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-4);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-6);
  }

  // With outlier
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.000001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->testUpdate_.outlierDetection_.setEnabledAll(true);
  this->testPredictAndUpdate_.outlierDetection_.setEnabledAll(true);
  this->testPrediction_.performPredictionUKF(filterState1,this->testPredictionMeas_,this->dt_);
  this->testUpdate_.performUpdateUKF(filterState1,this->testUpdateMeas_);
  this->testPrediction_.performPredictionUKF(filterState2,this->testPredictionMeas_,this->dt_);
  this->testPredictAndUpdate_.performUpdateUKF(filterState2,this->testUpdateMeas_);
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-4); // Increased difference comes because of different sigma points
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-6);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,2e-10);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-4);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-6);
  }
}

// Test comparePredictAndUpdate (including correlated noise)
TYPED_TEST(UpdateModelTest, comparePredictAndUpdate) {
  typename TestFixture::mtUpdateExample::mtFilterState filterState1;
  typename TestFixture::mtUpdateExample::mtFilterState filterState2;
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.0001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->testPrediction_.performPredictionEKF(filterState1,this->testPredictionMeas_,this->dt_);
  this->testPredictAndUpdate_.performUpdateEKF(filterState1,this->testUpdateMeas_);
  this->testPrediction_.performPredictionUKF(filterState2,this->testPredictionMeas_,this->dt_);
  this->testPredictAndUpdate_.performUpdateUKF(filterState2,this->testUpdateMeas_);
  typename TestFixture::mtUpdateExample::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,2e-2);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,2e-4);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-9);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-9);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,2e-2);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,8e-5);
  }

  // Negativ Control (Based on above)
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.0001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->testPrediction_.performPredictionEKF(filterState1,this->testPredictionMeas_,this->dt_);
  this->testPredictAndUpdate_.performUpdateEKF(filterState1,this->testUpdateMeas_);
  this->testPredictAndUpdate_.preupdnoiP_.block(0,0,3,3) = M3D::Identity()*0.00009;
  this->testPrediction_.performPredictionUKF(filterState2,this->testPredictionMeas_,this->dt_);
  this->testPredictAndUpdate_.performUpdateUKF(filterState2,this->testUpdateMeas_);
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_TRUE(dif.norm()>1e-1);
      ASSERT_TRUE((filterState1.cov_-filterState2.cov_).norm()>7e-5); // Bad discremination for nonlinear case
      break;
    case 1:
      ASSERT_TRUE(dif.norm()>1e-1);
      ASSERT_TRUE((filterState1.cov_-filterState2.cov_).norm()>1e-5);
      break;
    default:
      ASSERT_TRUE(dif.norm()>1e-1);
      ASSERT_TRUE((filterState1.cov_-filterState2.cov_).norm()>7e-5);
  }

  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.0001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->testPrediction_.performPredictionEKF(filterState1,this->testPredictionMeas_,this->dt_);
  this->testPredictAndUpdate_.performUpdateEKF(filterState1,this->testUpdateMeas_);
  this->testPrediction_.performPredictionUKF(filterState2,this->testPredictionMeas_,this->dt_);
  this->testPredictAndUpdate_.performUpdateUKF(filterState2,this->testUpdateMeas_);
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,2e-2);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,2e-4);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-9);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-9);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,2e-2);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,8e-5);
  }
}

// Test performUpdateLEKF1 (linearization point = prediction)
TYPED_TEST(UpdateModelTest, performUpdateLEKF1) {
  typename TestFixture::mtUpdateExample::mtFilterState filterState1;
  typename TestFixture::mtUpdateExample::mtFilterState filterState2;
  // Linearization point
  typename TestFixture::mtUpdateExample::mtState linState;
  linState = this->testState_;

  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.0001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  linState.boxMinus(filterState1.state_,filterState1.difVecLin_);
  this->testUpdate_.useSpecialLinearizationPoint_ = true;
  this->testUpdate_.performUpdateEKF(filterState1,this->testUpdateMeas_);
  this->testUpdate_.useSpecialLinearizationPoint_ = false;
  this->testUpdate_.performUpdateEKF(filterState2,this->testUpdateMeas_);
  typename TestFixture::mtUpdateExample::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,2e-2);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,8e-5);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-9);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-9);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,2e-2);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,8e-5);
  }
}

// Test performUpdateLEKF2 (linearization point <> prediction, but for linear case still the same result)
TYPED_TEST(UpdateModelTest, performUpdateLEKF2) {
  typename TestFixture::mtUpdateExample::mtFilterState filterState1;
  typename TestFixture::mtUpdateExample::mtFilterState filterState2;
  // Linearization point
  typename TestFixture::mtUpdateExample::mtState linState;
  linState = this->testState_;

  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.0001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  linState.boxMinus(filterState1.state_,filterState1.difVecLin_);
  this->testUpdate_.useSpecialLinearizationPoint_ = true;
  this->testUpdate_.performUpdateEKF(filterState1,this->testUpdateMeas_);
  this->testUpdate_.useSpecialLinearizationPoint_ = false;
  this->testUpdate_.performUpdateEKF(filterState2,this->testUpdateMeas_);
  typename TestFixture::mtUpdateExample::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      // No ASSERT
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-10);
      break;
    default:
      // No ASSERT
      break;
  }
}

// Test performUpdateLEKF3 (linearization point <> prediction, general)
TYPED_TEST(UpdateModelTest, performUpdateLEKF3) {
  this->testUpdate_.meas_ = this->testUpdateMeas_;
  // Linearization point
  typename TestFixture::mtUpdateExample::mtFilterState filterState;
  filterState.state_ = this->testState_;
  typename TestFixture::mtUpdateExample::mtState linState;
  unsigned int s = 2;
  linState.setRandom(s);

  filterState.cov_.setIdentity();
  Eigen::MatrixXd H((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtState::D_));
  this->testUpdate_.jacState(H,linState);
  Eigen::MatrixXd Hlin((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtState::D_));
  if(this->testUpdate_.useImprovedJacobian_){
    MXD boxMinusJac((int)TestFixture::mtState::D_,(int)TestFixture::mtState::D_);
    filterState.state_.boxMinusJac(linState,boxMinusJac);
    Hlin = H*boxMinusJac;
  } else {
    Hlin = H;
  }
  Eigen::MatrixXd Hn((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtNoise::D_));
  this->testUpdate_.jacNoise(Hn,linState);

  typename TestFixture::mtUpdateExample::mtInnovation y;
  this->testUpdate_.evalInnovationShort(y,linState);
  typename TestFixture::mtUpdateExample::mtInnovation yIdentity;
  yIdentity.setIdentity();
  typename TestFixture::mtUpdateExample::mtInnovation::mtDifVec innVector;

  typename TestFixture::mtUpdateExample::mtState stateUpdated;

  // Update
  MXD Py = Hlin*filterState.cov_*Hlin.transpose() + Hn*this->testUpdate_.updnoiP_*Hn.transpose();
  y.boxMinus(yIdentity,innVector);
  MXD Pyinv = Py.inverse();

  // Kalman Update
  Eigen::Matrix<double,TestFixture::mtUpdateExample::mtState::D_,TestFixture::mtUpdateExample::mtInnovation::D_> K = filterState.cov_*Hlin.transpose()*Pyinv;
  MXD updateCov = filterState.cov_ - K*Py*K.transpose();
  typename TestFixture::mtUpdateExample::mtState::mtDifVec updateVec;
  typename TestFixture::mtUpdateExample::mtState::mtDifVec difVecLin;
  linState.boxMinus(filterState.state_,difVecLin);
  updateVec = -K*(innVector-H*difVecLin);
  filterState.state_.boxPlus(updateVec,stateUpdated);

  linState.boxMinus(filterState.state_,filterState.difVecLin_);
  this->testUpdate_.useSpecialLinearizationPoint_ = true;
  this->testUpdate_.performUpdateEKF(filterState,this->testUpdateMeas_);
  this->testUpdate_.useSpecialLinearizationPoint_ = false;
  typename TestFixture::mtUpdateExample::mtState::mtDifVec dif;
  filterState.state_.boxMinus(stateUpdated,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-6);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-6);
      break;
  }
}

// Test performUpdateIEKF1 (for linear case still the same result)
TYPED_TEST(UpdateModelTest, performUpdateIEKF1) {
  typename TestFixture::mtUpdateExample::mtFilterState filterState1;
  typename TestFixture::mtUpdateExample::mtFilterState filterState2;
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.0001;
  filterState2.cov_ = filterState1.cov_;
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->testUpdate_.performUpdateIEKF(filterState1,this->testUpdateMeas_);
  this->testUpdate_.performUpdateEKF(filterState2,this->testUpdateMeas_);
  typename TestFixture::mtUpdateExample::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      // No ASSERT
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_).norm(),0.0,1e-10);
      break;
    default:
      // No ASSERT
      break;
  }
}

// Test performUpdateIEKF2 (general test)
TYPED_TEST(UpdateModelTest, performUpdateIEKF2) {
  this->testUpdate_.meas_ = this->testUpdateMeas_;
  typename TestFixture::mtUpdateExample::mtFilterState filterState;
  // Linearization point
  typename TestFixture::mtUpdateExample::mtState linState;
  linState = this->testState_;

  filterState.cov_.setIdentity();
  Eigen::MatrixXd H((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtState::D_));
  Eigen::MatrixXd Hn((int)(TestFixture::mtUpdateExample::mtInnovation::D_),(int)(TestFixture::mtUpdateExample::mtNoise::D_));

  typename TestFixture::mtUpdateExample::mtInnovation y;
  typename TestFixture::mtUpdateExample::mtInnovation yIdentity;
  yIdentity.setIdentity();
  typename TestFixture::mtUpdateExample::mtInnovation::mtDifVec innVector;

  typename TestFixture::mtUpdateExample::mtState stateUpdated;
  filterState.state_ = this->testState_;

  MXD Py;
  MXD Pyinv;
  Eigen::Matrix<double,TestFixture::mtUpdateExample::mtState::D_,TestFixture::mtUpdateExample::mtInnovation::D_> K;
  typename TestFixture::mtUpdateExample::mtState::mtDifVec updateVec;
  typename TestFixture::mtUpdateExample::mtState::mtDifVec difVecLin;

  double updateVecNorm = this->testUpdate_.updateVecNormTermination_;
  for(unsigned int i=0;i<this->testUpdate_.maxNumIteration_ & updateVecNorm>=this->testUpdate_.updateVecNormTermination_;i++){
    this->testUpdate_.jacState(H,linState);
    this->testUpdate_.jacNoise(Hn,linState);

    this->testUpdate_.evalInnovationShort(y,linState);

    // Update
    Py = H*filterState.cov_*H.transpose() + Hn*this->testUpdate_.updnoiP_*Hn.transpose();
    y.boxMinus(yIdentity,innVector);
    Pyinv = Py.inverse();

    // Kalman Update
    K = filterState.cov_*H.transpose()*Pyinv;
    filterState.state_.boxMinus(linState,difVecLin);
    updateVec = -K*(innVector+H*difVecLin)+difVecLin;
    linState.boxPlus(updateVec,linState);
    updateVecNorm = updateVec.norm();
  }
  stateUpdated = linState;
  MXD updateCov = filterState.cov_ - K*Py*K.transpose();

  this->testUpdate_.performUpdateIEKF(filterState,this->testUpdateMeas_);
  typename TestFixture::mtUpdateExample::mtState::mtDifVec dif;
  filterState.state_.boxMinus(stateUpdated,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-6);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-10);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-6);
      ASSERT_NEAR((filterState.cov_-updateCov).norm(),0.0,1e-6);
      break;
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
