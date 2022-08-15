#include "lightweight_filtering/TestClasses.hpp"
#include "lightweight_filtering/FilterBase.hpp"
#include "lightweight_filtering/common.hpp"
#include "gtest/gtest.h"
#include <assert.h>

using namespace LWFTest;

typedef ::testing::Types<
    NonlinearTest,
    LinearTest
> TestClasses;

// The fixture for testing class MeasurementTimeline
class MeasurementTimelineTest : public ::testing::Test {
 protected:
  MeasurementTimelineTest() {
    for(unsigned int i=0;i<N_;i++){
      times_[i] = i*i*0.1+i*0.3;
      values_[i] = (i*345665)%10+i*0.34;
    }
    timeline_.maxWaitTime_ = 1.0;
  }
  virtual ~MeasurementTimelineTest() {
  }
  LWF::MeasurementTimeline<double> timeline_;
  static const unsigned int N_ = 5;
  double times_[N_];
  double values_[N_];
};

// Test constructors
TEST_F(MeasurementTimelineTest, constructors) {
  LWF::MeasurementTimeline<double> timeline;
  ASSERT_EQ(timeline.maxWaitTime_,0.1);
}

// Test addMeas
TEST_F(MeasurementTimelineTest, addMeas) {
  for(int i=N_-1;i>=0;i--){ // Reverse for detecting FrontWarning
    timeline_.addMeas(values_[i],times_[i]);
    for(int j=N_-1;j>=i;j--){
      ASSERT_EQ(timeline_.measMap_.at(times_[j]),values_[j]);
    }
  }
}

// Test clean
TEST_F(MeasurementTimelineTest, clean) {
  for(unsigned int i=0;i<N_;i++){
    timeline_.addMeas(values_[i],times_[i]);
  }
  timeline_.clean(times_[N_-2]);
  ASSERT_EQ(timeline_.measMap_.size(),1);
  ASSERT_EQ(timeline_.measMap_.at(times_[N_-1]),values_[N_-1]);
}

// Test getNextTime
TEST_F(MeasurementTimelineTest, getNextTime) {
  for(unsigned int i=0;i<N_;i++){
    timeline_.addMeas(values_[i],times_[i]);
  }
  bool r;
  double nextTime;
  r = timeline_.getNextTime(times_[0]-1e-6,nextTime);
  ASSERT_EQ(r,true);
  ASSERT_EQ(nextTime,times_[0]);
  for(unsigned int i=0;i<N_-1;i++){
    r = timeline_.getNextTime(0.5*(times_[i]+times_[i+1]),nextTime);
    ASSERT_EQ(r,true);
    ASSERT_EQ(nextTime,times_[i+1]);
  }
  r = timeline_.getNextTime(times_[N_-1]+1e-6,nextTime);
  ASSERT_EQ(r,false);
}

// Test waitTime
TEST_F(MeasurementTimelineTest, waitTime) {
  for(unsigned int i=0;i<N_;i++){
    timeline_.addMeas(values_[i],times_[i]);
  }
  double time;
  for(unsigned int i=0;i<N_;i++){
    time = times_[i];
    timeline_.waitTime(times_[i],time);
    ASSERT_EQ(time,times_[i]);
  }
  time = times_[N_-1]+2.0;
  timeline_.waitTime(time,time);
  ASSERT_EQ(time,times_[N_-1]+1.0);
  time = times_[N_-1]+0.5;
  timeline_.waitTime(time,time);
  ASSERT_EQ(time,times_[N_-1]);
  time = times_[N_-1]-0.5;
  timeline_.waitTime(time,time);
  ASSERT_EQ(time,times_[N_-1]-0.5);
}

// Test getLastTime
TEST_F(MeasurementTimelineTest, getLastTime) {
  bool r;
  double lastTime;
  r = timeline_.getLastTime(lastTime);
  ASSERT_EQ(r,false);
  for(unsigned int i=0;i<N_;i++){
    timeline_.addMeas(values_[i],times_[i]);
  }
  r = timeline_.getLastTime(lastTime);
  ASSERT_EQ(r,true);
  ASSERT_EQ(lastTime,times_[N_-1]);
}

// Test hasMeasurementAt
TEST_F(MeasurementTimelineTest, hasMeasurementAt) {
  for(unsigned int i=0;i<N_;i++){
    timeline_.addMeas(values_[i],times_[i]);
  }
  bool r;
  for(unsigned int i=0;i<N_;i++){
    r = timeline_.hasMeasurementAt(times_[i]);
    ASSERT_EQ(r,true);
  }
  r = timeline_.hasMeasurementAt(0.5*(times_[0]+times_[1]));
  ASSERT_EQ(r,false);
}

// The fixture for testing class FilterBase
template<typename TestClass>
class FilterBaseTest : public ::testing::Test, public TestClass {
 protected:
  FilterBaseTest() {
    this->init(this->testFilterState_.state_,this->testUpdateMeas_,this->testPredictionMeas_);
    this->testFilter_.predictionTimeline_.maxWaitTime_ = 1.0;
    this->testFilter2_.predictionTimeline_.maxWaitTime_ = 1.0;
    std::get<0>(this->testFilter_.updateTimelineTuple_).maxWaitTime_ = 1.0;
    std::get<0>(this->testFilter2_.updateTimelineTuple_).maxWaitTime_ = 1.0;
    std::get<1>(this->testFilter_.updateTimelineTuple_).maxWaitTime_ = 0.0;
    std::get<1>(this->testFilter2_.updateTimelineTuple_).maxWaitTime_ = 0.0;
    switch(id_){
      case 0:
        this->testFilter_.readFromInfo("test_nonlinear.info");
        this->testFilter2_.readFromInfo("test_nonlinear.info");
        break;
      case 1:
        this->testFilter_.readFromInfo("test_linear.info");
        this->testFilter2_.readFromInfo("test_linear.info");
        break;
      default:
        this->testFilter_.readFromInfo("test_nonlinear.info");
        this->testFilter2_.readFromInfo("test_nonlinear.info");
    };
    this->testFilter_.reset();
    this->testFilter2_.reset();
  }
  virtual ~FilterBaseTest() {
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
  using TestClass::id_;
  LWF::FilterBase<mtPredictionExample,mtUpdateExample,mtPredictAndUpdateExample> testFilter_;
  LWF::FilterBase<mtPredictionExample,mtUpdateExample,mtPredictAndUpdateExample> testFilter2_;
  mtFilterState testFilterState_;
  typename TestClass::mtState::mtDifVec difVec_;
  mtUpdateMeas testUpdateMeas_;
  mtPredictionMeas testPredictionMeas_;
  const double dt_ = 0.1;
};

TYPED_TEST_CASE(FilterBaseTest, TestClasses);

// Test constructors
TYPED_TEST(FilterBaseTest, constructors) {
  LWF::FilterBase<typename TestFixture::mtPredictionExample,typename TestFixture::mtUpdateExample,typename TestFixture::mtPredictAndUpdateExample> testFilter;
}

// Test propertyHandler
TYPED_TEST(FilterBaseTest, propertyHandler) {
  // Generate parameters
  MXD prenoiP((int)TestFixture::mtPredictionExample::mtNoise::D_,(int)TestFixture::mtPredictionExample::mtNoise::D_);
  prenoiP.setZero();
  for(unsigned int i=0;i<TestFixture::mtPredictionExample::mtNoise::D_;i++){
    prenoiP(i,i) = 0.1*i*i+3.4*i+1.2;
  }
  MXD updnoiP((int)TestFixture::mtUpdateExample::mtNoise::D_,(int)TestFixture::mtUpdateExample::mtNoise::D_);
  updnoiP.setZero();
  for(unsigned int i=0;i<TestFixture::mtUpdateExample::mtNoise::D_;i++){
    updnoiP(i,i) = -0.1*i*i+3.2*i+1.1;
  }
  MXD updnoiP2((int)TestFixture::mtPredictAndUpdateExample::mtNoise::D_,(int)TestFixture::mtPredictAndUpdateExample::mtNoise::D_);
  updnoiP2.setZero();
  for(unsigned int i=0;i<TestFixture::mtPredictAndUpdateExample::mtNoise::D_;i++){
    updnoiP2(i,i) = -0.2*i*i+1.2*i+0.1;
  }
  Eigen::Matrix<double,TestFixture::mtPredictAndUpdateExample::mtPredictionNoise::D_,TestFixture::mtPredictAndUpdateExample::mtNoise::D_> preupdnoiP;
  preupdnoiP.setZero();
  for(unsigned int i=0;i<TestFixture::mtPredictAndUpdateExample::mtPredictionNoise::D_;i++){
    for(unsigned int j=0;j<TestFixture::mtPredictAndUpdateExample::mtNoise::D_;j++){
      preupdnoiP(i,j) = 0.3*i*i+0.2*i+3.1;
    }
  }
  MXD initP((int)TestFixture::mtState::D_,(int)TestFixture::mtState::D_);
  initP.setZero();
  for(unsigned int i=0;i<TestFixture::mtState::D_;i++){
    initP(i,i) = 0.5*i*i+3.1*i+3.2;
  }
  typename TestFixture::mtState initState;
  unsigned int s = 1;
  initState.setRandom(s);
  this->testFilter_.mPrediction_.prenoiP_ = prenoiP;
  std::get<0>(this->testFilter_.mUpdates_).updnoiP_ = updnoiP;
  std::get<1>(this->testFilter_.mUpdates_).updnoiP_ = updnoiP2;
  std::get<1>(this->testFilter_.mUpdates_).preupdnoiP_ = preupdnoiP;
  this->testFilter_.init_.cov_ = initP;
  this->testFilter_.init_.state_ = initState;

  // Write to file
  this->testFilter_.writeToInfo("test.info");

  // Set parameters zero
  this->testFilter_.mPrediction_.prenoiP_.setZero();
  std::get<0>(this->testFilter_.mUpdates_).updnoiP_.setZero();
  std::get<1>(this->testFilter_.mUpdates_).updnoiP_.setZero();
  std::get<1>(this->testFilter_.mUpdates_).preupdnoiP_.setZero();
  this->testFilter_.init_.cov_.setZero();
  this->testFilter_.init_.state_.setIdentity();

  // Read parameters from file and compare
  this->testFilter_.readFromInfo("test.info");
  ASSERT_NEAR((this->testFilter_.mPrediction_.prenoiP_-prenoiP).norm(),0.0,1e-6);
  ASSERT_NEAR((std::get<0>(this->testFilter_.mUpdates_).updnoiP_-updnoiP).norm(),0.0,1e-6);
  ASSERT_NEAR((std::get<1>(this->testFilter_.mUpdates_).updnoiP_-updnoiP2).norm(),0.0,1e-6);
//  ASSERT_NEAR((std::get<1>(this->testFilter_.mUpdates_).preupdnoiP_-preupdnoiP).norm(),0.0,1e-6);
  ASSERT_NEAR((this->testFilter_.init_.cov_-initP).norm(),0.0,1e-6);
  typename TestFixture::mtState::mtDifVec difVec;
  this->testFilter_.init_.state_.boxMinus(initState,difVec);
  ASSERT_NEAR(difVec.norm(),0.0,1e-6);
}

// Test updateSafe (Only for 1 update type (wait time set to zero for the other)), co-test getSafeTime() and setSafeWarningTime() and clean()
TYPED_TEST(FilterBaseTest, updateSafe) {
  double safeTime = 0.0;
  this->testFilter_.safeWarningTime_ = 0.1;

  std::cout << "Should print warning (2):" << std::endl;
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.1);
  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.1);
  ASSERT_TRUE(this->testFilter_.getSafeTime(safeTime));
  ASSERT_EQ(safeTime,0.1);
  this->testFilter_.updateSafe();
  ASSERT_EQ(this->testFilter_.safe_.t_,0.1);
  ASSERT_EQ(this->testFilter_.predictionTimeline_.measMap_.size(),1);
  ASSERT_EQ(std::get<0>(this->testFilter_.updateTimelineTuple_).measMap_.size(),1);

  std::cout << "Should print warning (2):" << std::endl;
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.1);
  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.1);
  ASSERT_TRUE(!this->testFilter_.getSafeTime(safeTime));
  ASSERT_EQ(safeTime,0.1);
  this->testFilter_.updateSafe();
  ASSERT_EQ(this->testFilter_.safe_.t_,0.1);
  ASSERT_EQ(this->testFilter_.predictionTimeline_.measMap_.size(),1);
  ASSERT_EQ(std::get<0>(this->testFilter_.updateTimelineTuple_).measMap_.size(),1);

  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.2);
  ASSERT_TRUE(!this->testFilter_.getSafeTime(safeTime));
  ASSERT_EQ(safeTime,0.1);
  this->testFilter_.updateSafe();
  ASSERT_EQ(this->testFilter_.safe_.t_,0.1);
  ASSERT_EQ(this->testFilter_.predictionTimeline_.measMap_.size(),1);
  ASSERT_EQ(std::get<0>(this->testFilter_.updateTimelineTuple_).measMap_.size(),2);

  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.2);
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.3);
  ASSERT_TRUE(this->testFilter_.getSafeTime(safeTime));
  ASSERT_EQ(safeTime,0.2);
  this->testFilter_.updateSafe();
  ASSERT_EQ(this->testFilter_.safe_.t_,0.2);
  ASSERT_EQ(this->testFilter_.predictionTimeline_.measMap_.size(),1);
  ASSERT_EQ(std::get<0>(this->testFilter_.updateTimelineTuple_).measMap_.size(),1);

  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.3);
  ASSERT_TRUE(this->testFilter_.getSafeTime(safeTime));
  ASSERT_EQ(safeTime,0.3);
  this->testFilter_.updateSafe();
  ASSERT_EQ(this->testFilter_.safe_.t_,0.3);
  ASSERT_EQ(this->testFilter_.predictionTimeline_.measMap_.size(),1);
  ASSERT_EQ(std::get<0>(this->testFilter_.updateTimelineTuple_).measMap_.size(),1);
}

// Test updateFront
TYPED_TEST(FilterBaseTest, updateFront) {
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.1);
  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.1);
  ASSERT_TRUE(this->testFilter_.gotFrontWarning_==false);
  this->testFilter_.updateFront(0.5);
  ASSERT_TRUE(this->testFilter_.gotFrontWarning_==false);
  ASSERT_EQ(this->testFilter_.safe_.t_,0.1);
  ASSERT_EQ(this->testFilter_.front_.t_,0.5);

  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.2);
  ASSERT_TRUE(this->testFilter_.gotFrontWarning_==true);
  this->testFilter_.updateFront(0.2);
  ASSERT_TRUE(this->testFilter_.gotFrontWarning_==false);
  ASSERT_EQ(this->testFilter_.safe_.t_,0.1);
  ASSERT_EQ(this->testFilter_.front_.t_,0.2);

  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.2);
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.3);
  ASSERT_TRUE(this->testFilter_.gotFrontWarning_==true);
  this->testFilter_.updateFront(0.3);
  ASSERT_TRUE(this->testFilter_.gotFrontWarning_==false);
  ASSERT_EQ(this->testFilter_.safe_.t_,0.2);
  ASSERT_EQ(this->testFilter_.front_.t_,0.3);

  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.3);
  ASSERT_TRUE(this->testFilter_.gotFrontWarning_==true);
  this->testFilter_.updateFront(0.3);
  ASSERT_TRUE(this->testFilter_.gotFrontWarning_==false);
  ASSERT_EQ(this->testFilter_.safe_.t_,0.3);
  ASSERT_EQ(this->testFilter_.front_.t_,0.3);
}

// Test high level logic
TYPED_TEST(FilterBaseTest, highlevel) {
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.1);
  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.1);
  this->testFilter_.updateSafe();
  this->testFilter2_.addPredictionMeas(this->testPredictionMeas_,0.1);
  this->testFilter2_.template addUpdateMeas<0>(this->testUpdateMeas_,0.1);
  this->testFilter2_.updateSafe();

  this->testFilter2_.safe_.state_.boxMinus(this->testFilter_.safe_.state_,this->difVec_);
  ASSERT_EQ(this->testFilter_.safe_.t_,this->testFilter2_.safe_.t_);
  ASSERT_NEAR(this->difVec_.norm(),0.0,1e-6);
  ASSERT_NEAR((this->testFilter2_.safe_.cov_-this->testFilter_.safe_.cov_).norm(),0.0,1e-6);


  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.2);
  this->testFilter_.updateFront(0.2);
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.2);
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.3);
  this->testFilter_.updateFront(0.3);
  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.3);
  this->testFilter_.updateFront(0.3);

  this->testFilter2_.template addUpdateMeas<0>(this->testUpdateMeas_,0.2);
  this->testFilter2_.addPredictionMeas(this->testPredictionMeas_,0.2);
  this->testFilter2_.addPredictionMeas(this->testPredictionMeas_,0.3);
  this->testFilter2_.template addUpdateMeas<0>(this->testUpdateMeas_,0.3);
  this->testFilter2_.updateSafe();

  this->testFilter2_.safe_.state_.boxMinus(this->testFilter_.safe_.state_,this->difVec_);
  ASSERT_EQ(this->testFilter_.safe_.t_,this->testFilter2_.safe_.t_);
  ASSERT_NEAR(this->difVec_.norm(),0.0,1e-6);
  ASSERT_NEAR((this->testFilter2_.safe_.cov_-this->testFilter_.safe_.cov_).norm(),0.0,1e-6);
}

// Test high level logic 2: coupled
TYPED_TEST(FilterBaseTest, highlevel2) {
  std::get<1>(this->testFilter_.mUpdates_).preupdnoiP_.block(0,0,3,3) = M3D::Identity()*0.00009;
  std::get<1>(this->testFilter2_.mUpdates_).preupdnoiP_.block(0,0,3,3) = M3D::Identity()*0.00009;
  this->testFilterState_.state_ = this->testFilter_.safe_.state_;
  this->testFilterState_.cov_ = this->testFilter_.safe_.cov_;
  std::get<1>(this->testFilter_.updateTimelineTuple_).maxWaitTime_ = 1.0;
  std::get<1>(this->testFilter2_.updateTimelineTuple_).maxWaitTime_ = 1.0;
  std::get<0>(this->testFilter_.updateTimelineTuple_).maxWaitTime_ = 0.0;
  std::get<0>(this->testFilter2_.updateTimelineTuple_).maxWaitTime_ = 0.0;
  // TestFilter
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.1);
  this->testFilter_.template addUpdateMeas<1>(this->testUpdateMeas_,0.1);
  this->testFilter_.updateSafe();
  // TestFilter2
  this->testFilter2_.addPredictionMeas(this->testPredictionMeas_,0.1);
  this->testFilter2_.template addUpdateMeas<1>(this->testUpdateMeas_,0.1);
  this->testFilter2_.updateSafe();
  // Direct
  this->testFilter_.mPrediction_.performPredictionEKF(this->testFilterState_,this->testPredictionMeas_,0.1);
  std::get<1>(this->testFilter_.mUpdates_).performUpdateEKF(this->testFilterState_,this->testUpdateMeas_);

  // Compare
  this->testFilter2_.safe_.state_.boxMinus(this->testFilter_.safe_.state_,this->difVec_);
  ASSERT_EQ(this->testFilter_.safe_.t_,this->testFilter2_.safe_.t_);
  ASSERT_NEAR(this->difVec_.norm(),0.0,1e-6);
  ASSERT_NEAR((this->testFilter2_.safe_.cov_-this->testFilter_.safe_.cov_).norm(),0.0,1e-6);
  this->testFilter_.safe_.state_.boxMinus(this->testFilterState_.state_,this->difVec_);
  ASSERT_NEAR(this->difVec_.norm(),0.0,1e-6);
  ASSERT_NEAR((this->testFilter_.safe_.cov_-this->testFilterState_.cov_).norm(),0.0,1e-6);

  // TestFilter
  this->testFilter_.template addUpdateMeas<1>(this->testUpdateMeas_,0.2);
  this->testFilter_.updateFront(0.2);
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.2);
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.3);
  this->testFilter_.updateFront(0.3);
  this->testFilter_.template addUpdateMeas<1>(this->testUpdateMeas_,0.3);
  this->testFilter_.updateFront(0.3);
  // TestFilter2
  this->testFilter2_.template addUpdateMeas<1>(this->testUpdateMeas_,0.2);
  this->testFilter2_.addPredictionMeas(this->testPredictionMeas_,0.2);
  this->testFilter2_.addPredictionMeas(this->testPredictionMeas_,0.3);
  this->testFilter2_.template addUpdateMeas<1>(this->testUpdateMeas_,0.3);
  this->testFilter2_.updateSafe();
  // Direct
  this->testFilter_.mPrediction_.performPredictionEKF(this->testFilterState_,this->testPredictionMeas_,0.1);
  std::get<1>(this->testFilter_.mUpdates_).performUpdateEKF(this->testFilterState_,this->testUpdateMeas_);
  this->testFilter_.mPrediction_.performPredictionEKF(this->testFilterState_,this->testPredictionMeas_,0.1);
  std::get<1>(this->testFilter_.mUpdates_).performUpdateEKF(this->testFilterState_,this->testUpdateMeas_);

  // Compare
  this->testFilter2_.safe_.state_.boxMinus(this->testFilter_.safe_.state_,this->difVec_);
  ASSERT_EQ(this->testFilter_.safe_.t_,this->testFilter2_.safe_.t_);
  ASSERT_NEAR(this->difVec_.norm(),0.0,1e-6);
  ASSERT_NEAR((this->testFilter2_.safe_.cov_-this->testFilter_.safe_.cov_).norm(),0.0,1e-6);
  this->testFilter_.safe_.state_.boxMinus(this->testFilterState_.state_,this->difVec_);
  ASSERT_NEAR(this->difVec_.norm(),0.0,1e-6);
  ASSERT_NEAR((this->testFilter_.safe_.cov_-this->testFilterState_.cov_).norm(),0.0,1e-6);
}

// Test high level logic 3: merged
TYPED_TEST(FilterBaseTest, highlevel3) {
  this->testFilterState_.state_ = this->testFilter2_.safe_.state_;
  this->testFilterState_.cov_ = this->testFilter2_.safe_.cov_;
  // TestFilter and direct method
  this->testFilterState_.usePredictionMerge_ = true;
  this->testFilter_.safe_.usePredictionMerge_ = true;
  this->testFilter2_.safe_.usePredictionMerge_ = true;
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.1);
  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.1);
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.2);
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.3);
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.4);
  this->testFilter_.addPredictionMeas(this->testPredictionMeas_,0.5);
  this->testFilter_.template addUpdateMeas<0>(this->testUpdateMeas_,0.5);
    this->testFilter_.mPrediction_.performPredictionEKF(this->testFilterState_,this->testPredictionMeas_,0.1);
    std::get<0>(this->testFilter_.mUpdates_).performUpdateEKF(this->testFilterState_,this->testUpdateMeas_);
    this->testFilter_.mPrediction_.predictMergedEKF(this->testFilterState_,0.5,this->testFilter_.predictionTimeline_.measMap_);
    std::get<0>(this->testFilter_.mUpdates_).performUpdateEKF(this->testFilterState_,this->testUpdateMeas_);
  this->testFilter_.updateSafe();
  // TestFilter2
  this->testFilter2_.addPredictionMeas(this->testPredictionMeas_,0.1);
  this->testFilter2_.template addUpdateMeas<0>(this->testUpdateMeas_,0.1);
  this->testFilter2_.updateSafe();
  this->testFilter2_.addPredictionMeas(this->testPredictionMeas_,0.2);
  this->testFilter2_.updateSafe();
  this->testFilter2_.addPredictionMeas(this->testPredictionMeas_,0.3);
  this->testFilter2_.updateSafe();
  this->testFilter2_.addPredictionMeas(this->testPredictionMeas_,0.4);
  this->testFilter2_.updateSafe();
  this->testFilter2_.addPredictionMeas(this->testPredictionMeas_,0.5);
  this->testFilter2_.updateSafe();
  this->testFilter2_.template addUpdateMeas<0>(this->testUpdateMeas_,0.5);
  this->testFilter2_.updateSafe();

  // Compare
  this->testFilter2_.safe_.state_.boxMinus(this->testFilter_.safe_.state_,this->difVec_);
  ASSERT_EQ(this->testFilter_.safe_.t_,this->testFilter2_.safe_.t_);
  ASSERT_NEAR(this->difVec_.norm(),0.0,1e-6);
  ASSERT_NEAR((this->testFilter2_.safe_.cov_-this->testFilter_.safe_.cov_).norm(),0.0,1e-6);
  this->testFilter_.safe_.state_.boxMinus(this->testFilterState_.state_,this->difVec_);
  ASSERT_NEAR(this->difVec_.norm(),0.0,1e-6);
  ASSERT_NEAR((this->testFilter_.safe_.cov_-this->testFilterState_.cov_).norm(),0.0,1e-6);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
