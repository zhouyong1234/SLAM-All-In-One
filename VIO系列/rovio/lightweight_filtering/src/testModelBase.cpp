#include "lightweight_filtering/State.hpp"
#include "lightweight_filtering/ModelBase.hpp"
#include "lightweight_filtering/common.hpp"
#include "gtest/gtest.h"
#include <assert.h>

class Input: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,2>,LWF::TH_multiple_elements<LWF::QuaternionElement,2>>{
 public:
  enum StateNames{
    v0, v1, q0, q1
  };
  Input(){};
  ~Input(){};
};
class Output: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
  enum StateNames{
    v0, q0
  };
  Output(){};
  ~Output(){};
};
class Meas: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
  enum StateNames{
    v0, q0
  };
  Meas(){};
  ~Meas(){};
};
class Noise: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,2>>{
 public:
  enum StateNames{
    v0, v1
  };
  Noise(){};
  ~Noise(){};
};
class ModelExample: public LWF::ModelBase<ModelExample,Output,Input,Meas,Noise>{
 public:
  ModelExample(){};
  ~ModelExample(){};
  void eval_(Output& output, const mtInputTuple& inputs, double dt) const{
    const Input& input = std::get<0>(inputs);
    const Meas& meas = std::get<1>(inputs);
    const Noise& noise = std::get<2>(inputs);
    QPD dQ = dQ.exponentialMap(noise.get<Noise::v1>());
    output.get<Output::v0>() = (input.get<Input::q0>().inverted()*input.get<Input::q1>()).rotate(input.get<Input::v1>())-input.get<Input::v0>()+noise.get<Noise::v0>()-meas.get<Meas::v0>();
    output.get<Output::q0>() = meas.get<Meas::q0>().inverted()*dQ*input.get<Input::q1>().inverted()*input.get<Input::q0>();
  }
  template<int i,typename std::enable_if<i==0>::type* = nullptr>
  void jacInput_(Eigen::MatrixXd& J, const mtInputTuple& inputs, double dt) const{
    const Input& input = std::get<0>(inputs);
    const Meas& meas = std::get<1>(inputs);
    const Noise& noise = std::get<2>(inputs);
    QPD dQ = dQ.exponentialMap(noise.get<Noise::v1>());
    J.setZero();
    J.block<3,3>(Output::getId<Output::v0>(),Input::getId<Input::v0>()) = -M3D::Identity();
    J.block<3,3>(Output::getId<Output::v0>(),Input::getId<Input::v1>()) = MPD(input.get<Input::q0>().inverted()*input.get<Input::q1>()).matrix();
    J.block<3,3>(Output::getId<Output::v0>(),Input::getId<Input::q0>()) = gSM((input.get<Input::q0>().inverted()*input.get<Input::q1>()).rotate(input.get<Input::v1>()))*MPD(input.get<Input::q0>().inverted()).matrix();
    J.block<3,3>(Output::getId<Output::v0>(),Input::getId<Input::q1>()) = -gSM((input.get<Input::q0>().inverted()*input.get<Input::q1>()).rotate(input.get<Input::v1>()))*MPD(input.get<Input::q0>().inverted()).matrix();
    J.block<3,3>(Output::getId<Output::q0>(),Input::getId<Input::q0>()) = MPD(meas.get<Meas::q0>().inverted()*dQ*input.get<Input::q1>().inverted()).matrix();
    J.block<3,3>(Output::getId<Output::q0>(),Input::getId<Input::q1>()) = -MPD(meas.get<Meas::q0>().inverted()*dQ*input.get<Input::q1>().inverted()).matrix();
  }
  template<int i,typename std::enable_if<i==1>::type* = nullptr>
  void jacInput_(Eigen::MatrixXd& J, const mtInputTuple& inputs, double dt) const{
    // Not done
  }
  template<int i,typename std::enable_if<i==2>::type* = nullptr>
  void jacInput_(Eigen::MatrixXd& J, const mtInputTuple& inputs, double dt) const{
    const Input& input = std::get<0>(inputs);
    const Meas& meas = std::get<1>(inputs);
    const Noise& noise = std::get<2>(inputs);
    QPD dQ = dQ.exponentialMap(noise.get<Noise::v1>());
    J.setZero();
    J.block<3,3>(Output::getId<Output::v0>(),0) = M3D::Identity();
    J.block<3,3>(Output::getId<Output::q0>(),3) = MPD(meas.get<Meas::q0>().inverted()).matrix()*Lmat(noise.get<Noise::v1>());
  }
};

// The fixture for testing class PredictionModel
class ModelBaseTest : public ::testing::Test {
 protected:
  ModelBaseTest() {
    unsigned int s = 0;
    testNoise_.setRandom(s);
    testInput_.setRandom(s);
    testMeas_.setRandom(s);
  }
  virtual ~ModelBaseTest(){}
  ModelExample model_;
  Input testInput_;
  Meas testMeas_;
  Noise testNoise_;
};

// Test finite difference Jacobians
TEST_F(ModelBaseTest, FDjacobians) {
  Eigen::MatrixXd F((int)(Output::D_),(int)(Input::D_));
  Eigen::MatrixXd F_FD((int)(Output::D_),(int)(Input::D_));
  model_.template jacInput<0>(F,std::forward_as_tuple(testInput_,testMeas_,testNoise_),0.1);
  model_.template jacInputFD<0>(F_FD,std::forward_as_tuple(testInput_,testMeas_,testNoise_),0.1,0.0000001);
  ASSERT_NEAR((F-F_FD).norm(),0.0,1e-5);
  Eigen::MatrixXd H((int)(Output::D_),(int)(Noise::D_));
  Eigen::MatrixXd H_FD((int)(Output::D_),(int)(Noise::D_));
  model_.template jacInput<2>(H,std::forward_as_tuple(testInput_,testMeas_,testNoise_),0.1);
  model_.template jacInputFD<2>(H_FD,std::forward_as_tuple(testInput_,testMeas_,testNoise_),0.1,0.0000001);
  ASSERT_NEAR((H-H_FD).norm(),0.0,1e-5);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
