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
class GIFPredictionModelTest : public ::testing::Test, public TestClass {
 protected:
  GIFPredictionModelTest() {
    unsigned int s = 1;
    testState_.setRandom(s);
    testPredictionMeas_.setRandom(s);
    updateMeas_.setRandom(s);
    TestClass::mergePredictionAndUpdateMeas(GIFMeasWithUpdate_,testPredictionMeas_,updateMeas_);
    predictionExample_.prenoiP_.setIdentity();
    predictionExample_.prenoiP_ = 0.01*predictionExample_.prenoiP_;
    predictionExample_.prenoiP_.block(0,0,mtState::D_,mtState::D_) = 0.25*predictionExample_.prenoiP_.block(0,0,mtState::D_,mtState::D_);
    updateExample_.updnoiP_.setIdentity();
    updateExample_.updnoiP_ = 0.04*updateExample_.updnoiP_;
    GIFpredictionExample_.noiP_ = predictionExample_.prenoiP_;
    GIFpredictionExampleWithUpdate_.noiP_.block(0,0,mtPredictionNoise::D_,mtPredictionNoise::D_) = predictionExample_.prenoiP_;
    GIFpredictionExampleWithUpdate_.noiP_.block(mtPredictionNoise::D_,mtPredictionNoise::D_,mtUpdateNoise::D_,mtUpdateNoise::D_) = updateExample_.updnoiP_;
  }
  virtual ~GIFPredictionModelTest() {
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
  using typename TestClass::mtGIFPredictionExample;
  using typename TestClass::mtGIFPredictionExampleWithUpdate;
  using typename TestClass::mtGIFMeasWithUpdate;
  mtGIFPredictionExample GIFpredictionExample_;
  mtPredictionExample predictionExample_;
  mtState testState_;
  mtPredictionMeas testPredictionMeas_;
  mtGIFPredictionExampleWithUpdate GIFpredictionExampleWithUpdate_;
  mtGIFMeasWithUpdate GIFMeasWithUpdate_;
  mtUpdateMeas updateMeas_;
  mtUpdateExample updateExample_;
  const double dt_ = 0.1;
//  std::map<double,mtPredictionMeas> measMap_;
};

TYPED_TEST_CASE(GIFPredictionModelTest, TestClasses);

// Test constructors and Jacobians
TYPED_TEST(GIFPredictionModelTest, constructors) {
  typename TestFixture::mtFilterState filterState;
  double dt = 0.1;
  this->GIFpredictionExample_.performPrediction(filterState,this->testPredictionMeas_,dt);
  typename TestFixture::mtGIFPredictionExample testPrediction;
  ASSERT_EQ((testPrediction.noiP_-Eigen::Matrix<double,TestFixture::mtState::D_,TestFixture::mtState::D_>::Identity()*0.0001).norm(),0.0);
  ASSERT_TRUE(this->GIFpredictionExample_.testPredictionJacs(1e-8,1e-5,0.1));
  ASSERT_TRUE(this->GIFpredictionExampleWithUpdate_.testPredictionJacs(1e-8,1e-5,0.1));
}

// Test comparePredict
TYPED_TEST(GIFPredictionModelTest, comparePredict) {
  typename TestFixture::mtFilterState filterState1;
  typename TestFixture::mtFilterState filterState2;
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.01;
  filterState2.cov_ = filterState1.cov_.inverse();
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->predictionExample_.performPredictionEKF(filterState1,this->testPredictionMeas_,this->dt_);
  this->GIFpredictionExample_.performPrediction(filterState2,this->testPredictionMeas_,this->dt_);
  typename TestFixture::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-5);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_.inverse()).norm(),0.0,1e-6);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-9);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_.inverse()).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-5);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_.inverse()).norm(),0.0,1e-6);
  };
}

// Test comparePredictWithUpdate
TYPED_TEST(GIFPredictionModelTest, comparePredictWithUpdate) {
  typename TestFixture::mtFilterState filterState1;
  typename TestFixture::mtFilterState filterState2;
  filterState1.cov_.setIdentity();
  filterState1.cov_ = filterState1.cov_*0.01;
  filterState2.cov_ = filterState1.cov_.inverse();
  filterState1.state_ = this->testState_;
  filterState2.state_ = this->testState_;
  this->predictionExample_.performPredictionEKF(filterState1,this->testPredictionMeas_,this->dt_);
  this->updateExample_.performUpdateEKF(filterState1,this->updateMeas_);
  this->GIFpredictionExampleWithUpdate_.performPrediction(filterState2,this->GIFMeasWithUpdate_,this->dt_);
  typename TestFixture::mtState::mtDifVec dif;
  filterState1.state_.boxMinus(filterState2.state_,dif);
  switch(TestFixture::id_){
    case 0:
      ASSERT_NEAR(dif.norm(),0.0,1e-5);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_.inverse()).norm(),0.0,1e-6);
      break;
    case 1:
      ASSERT_NEAR(dif.norm(),0.0,1e-9);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_.inverse()).norm(),0.0,1e-10);
      break;
    default:
      ASSERT_NEAR(dif.norm(),0.0,1e-5);
      ASSERT_NEAR((filterState1.cov_-filterState2.cov_.inverse()).norm(),0.0,1e-6);
  };
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
