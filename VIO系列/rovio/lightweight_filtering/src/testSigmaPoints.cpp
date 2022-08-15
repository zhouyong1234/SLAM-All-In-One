#include "lightweight_filtering/SigmaPoints.hpp"
#include "lightweight_filtering/State.hpp"
#include "lightweight_filtering/common.hpp"
#include "gtest/gtest.h"
#include <assert.h>

// The fixture for testing class ScalarElement.
class SigmaPointTest : public ::testing::Test {
 protected:
  SigmaPointTest(): P_((int)(mtState::D_),(int)(mtState::D_)), Qmat_((int)(mtState::D_),(int)(mtState::D_)){
    sigmaPoints_.computeParameter(1e-3,2.0,0.0);
    sigmaPointsVector_.computeParameter(1e-3,2.0,0.0);
    mean_.template get<0>(0) = 4.5;
    for(int i=1;i<S_;i++){
      mean_.template get<0>(i) = mean_.template get<0>(i-1) + i*i*46.2;
    }
    mean_.template get<1>(0) << 2.1, -0.2, -1.9;
    for(int i=1;i<V_;i++){
      mean_.template get<1>(i) = mean_.template get<1>(i-1) + V3D(0.3,10.9,2.3);
    }
    mean_.template get<2>(0) = QPD(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));
    for(int i=1;i<Q_;i++){
      mean_.template get<2>(i) = mean_.template get<2>(i-1).boxPlus(mean_.template get<1>(i-1));
    }
    // Easy way to obtain a pseudo random positive definite matrix
    P_ = mtState::D_*Eigen::MatrixXd::Identity((int)(mtState::D_),(int)(mtState::D_));
    double randValue;
    for(int i=0;i<mtState::D_;i++){
      for(int j=i;j<mtState::D_;j++){
        randValue = (cos((double)(123456*(i+j+1)))+1.0)/2.0;
        P_(i,j) += randValue;
        P_(j,i) += randValue;
      }
    }
    Qmat_ = mtState::D_*Eigen::MatrixXd::Identity((int)(mtState::D_),(int)(mtState::D_));
    for(int i=0;i<mtState::D_;i++){
      for(int j=i;j<mtState::D_;j++){
        randValue = (cos((double)(123456*(i+j+1)))+1.0)/2.0;
        Qmat_(i,j) += randValue;
        Qmat_(j,i) += randValue;
      }
    }
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Qmat_);
    Qmat_ = qr.householderQ();
  }
  ~SigmaPointTest() {
  }
  static const unsigned int S_ = 4;
  static const unsigned int V_ = 3;
  static const unsigned int Q_ = 2;
  static const unsigned int N_ = 2*(S_+3*(V_+Q_))+1;
  static const unsigned int O_ = 2;
  static const unsigned int L_ = N_+O_+2;
  typedef LWF::State<LWF::ArrayElement<LWF::ScalarElement,S_>,LWF::ArrayElement<LWF::VectorElement<3>,V_>,LWF::ArrayElement<LWF::QuaternionElement,Q_>> mtState;
  typedef LWF::VectorElement<3> mtElementVector;
  typedef mtState::mtDifVec mtDifVec;
  typedef LWF::SigmaPoints<mtState,N_,L_,O_> mtSigmaPoints;
  mtSigmaPoints sigmaPoints_;
  LWF::SigmaPoints<mtElementVector,L_,L_,0> sigmaPointsVector_;
  mtState mean_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Qmat_;
  double alpha_ = 1e-3;
  double beta_ = 2.0;
  double kappa_ = 0.0;
};

// Test constructors
TEST_F(SigmaPointTest, constructors) {
  LWF::SigmaPoints<mtState,N_,L_,O_> sigmaPoints;
  ASSERT_TRUE(sigmaPoints.N_==N_);
  ASSERT_TRUE(sigmaPoints.L_==L_);
  ASSERT_TRUE(sigmaPoints.O_==O_);
}

// Test computeParameter
TEST_F(SigmaPointTest, computeParameter) {
  sigmaPoints_.computeParameter(alpha_,beta_,kappa_);
  const unsigned int D = (L_-1)/2;
  double lambda = alpha_*alpha_*(D+kappa_)-D;
  double gamma = sqrt(lambda + D);
  double wm = 1/(2*(D+lambda));
  double wc = wm;
  double wc0 = lambda/(D+lambda)+(1-alpha_*alpha_+beta_);
  ASSERT_EQ(sigmaPoints_.wm_,wm);
  ASSERT_EQ(sigmaPoints_.wc_,wc);
  ASSERT_EQ(sigmaPoints_.wc0_,wc0);
  ASSERT_EQ(sigmaPoints_.gamma_,gamma);
}

// Test computeFromGaussian, getMean, getCovariance, computeFromZeroMeanGaussian
TEST_F(SigmaPointTest, computeFromGaussianPlusPlus) {
  // computeFromGaussian
  sigmaPoints_.computeFromGaussian(mean_,P_);

  // Check mean is same
  mtState mean;
  sigmaPoints_.getMean(mean);
  mtDifVec vec;
  mean.boxMinus(mean_,vec);
  ASSERT_NEAR(vec.norm(),0.0,1e-8);

  // Check covariance is same
  Eigen::MatrixXd P((int)(mtState::D_),(int)(mtState::D_));
  sigmaPoints_.getCovarianceMatrix(mean,P);
  ASSERT_NEAR((P-P_).norm(),0.0,1e-8);

  // computeFromZeroMeanGaussian
  sigmaPoints_.computeFromZeroMeanGaussian(P_);

  // Check mean is same
  sigmaPoints_.getMean(mean);
  mean.boxMinus(mtState::Identity(),vec);
  ASSERT_NEAR(vec.norm(),0.0,1e-8);

  // Check covariance is same
  sigmaPoints_.getCovarianceMatrix(mean,P);
  ASSERT_NEAR((P-P_).norm(),0.0,1e-8);

  // Test with semidefinite matrix
  Eigen::MatrixXd Psemi = P_;
  Psemi.col(1) = Psemi.col(0);
  Psemi.row(1) = Psemi.row(0);
  sigmaPoints_.computeFromGaussian(mean_,Psemi);
  sigmaPoints_.getMean(mean);
  mean.boxMinus(mean_,vec);
  ASSERT_NEAR(vec.norm(),0.0,1e-8);
  sigmaPoints_.getCovarianceMatrix(mean,P);
  ASSERT_NEAR((P-Psemi).norm(),0.0,1e-8);
}

// Test computeFromGaussian with W matrix
TEST_F(SigmaPointTest, computeFromGaussianQ) {
  // computeFromGaussian
  sigmaPoints_.computeFromGaussian(mean_,P_,Qmat_);

  // Check mean is same
  mtState mean;
  sigmaPoints_.getMean(mean);
  mtDifVec vec;
  mean.boxMinus(mean_,vec);
  ASSERT_NEAR(vec.norm(),0.0,1e-8);

  // Check covariance is same
  Eigen::MatrixXd P((int)(mtState::D_),(int)(mtState::D_));
  sigmaPoints_.getCovarianceMatrix(mean,P);
  ASSERT_NEAR((P-P_).norm(),0.0,1e-8);
}

// Test getMean, getCovariance2
TEST_F(SigmaPointTest, getCovariance2) {
  // computeFromGaussian
  sigmaPoints_.computeFromGaussian(mean_,P_);

  // Apply simple linear transformation
  for(int i=0;i<L_;i++){
    sigmaPointsVector_(i).v_ = sigmaPoints_(i).template get<1>(0)*2.45+V3D::Ones()*sigmaPoints_(i).template get<0>(0)*0.51;
  }

  Eigen::MatrixXd M((int)(mtElementVector::D_),(int)(mtState::D_));
  sigmaPointsVector_.getCovarianceMatrix(sigmaPoints_,M);
  Eigen::MatrixXd H((int)(mtElementVector::D_),(int)(mtState::D_)); // Jacobian of linear transformation
  H.setZero();
  H.block(0,S_,3,3) = M3D::Identity()*2.45;
  H.block(0,0,3,1) = V3D::Ones()*0.51;
  Eigen::Matrix<double,mtElementVector::D_,mtState::D_> Mref = H*P_;
  for(int i=0;i<mtElementVector::D_;i++){
    for(int j=0;j<mtState::D_;j++){
      ASSERT_NEAR(Mref(i,j),M(i,j),1e-8);
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
