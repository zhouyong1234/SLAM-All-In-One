/*
 * TestClasses.hpp
 *
 *  Created on: Feb 9, 2014
 *      Author: Bloeschm
 */

#ifndef LWF_TestClasses_HPP_
#define LWF_TestClasses_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/State.hpp"
#include "lightweight_filtering/FilterState.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/Prediction.hpp"
#include "lightweight_filtering/GIFPrediction.hpp"

namespace LWFTest{

namespace Nonlinear{

class State: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,4>,LWF::QuaternionElement>{
 public:
  enum StateNames {
    POS,
    VEL,
    ACB,
    GYB,
    ATT
  };
  State(){
    getName<0>() = "pos";
    getName<1>() = "vel";
    getName<2>() = "acb";
    getName<3>() = "gyb";
    getName<4>() = "att";
  }
  virtual ~State(){};
};
class UpdateMeas: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
  enum StateNames {
    POS,
    ATT
  };
  UpdateMeas(){};
  virtual ~UpdateMeas(){};
};
class UpdateNoise: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,2>>{
 public:
  enum StateNames {
    POS,
    ATT
  };
  UpdateNoise(){};
  virtual ~UpdateNoise(){};
};
class Innovation: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
  enum StateNames {
    POS,
    ATT
  };
  Innovation(){};
  virtual ~Innovation(){};
};
class PredictionNoise: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,5>>{
 public:
  enum StateNames {
    POS,
    VEL,
    ACB,
    GYB,
    ATT
  };
  PredictionNoise(){};
  virtual ~PredictionNoise(){};
};
class PredictionMeas: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,2>>{
 public:
  enum StateNames {
    ACC,
    GYR
  };
  PredictionMeas(){};
  virtual ~PredictionMeas(){};
};
class OutlierDetectionExample: public LWF::OutlierDetection<LWF::ODEntry<0,3,1>>{
 public:
  virtual ~OutlierDetectionExample(){};
};
class FilterState: public LWF::FilterState<State,PredictionMeas,PredictionNoise,UpdateNoise::D_>{
 public:
  virtual ~FilterState(){};
};

class UpdateExample: public LWF::Update<Innovation,FilterState,UpdateMeas,UpdateNoise,OutlierDetectionExample,false>{
 public:
  using mtState = LWF::Update<Innovation,FilterState,UpdateMeas,UpdateNoise,OutlierDetectionExample,false>::mtState;
  typedef UpdateMeas mtMeas;
  typedef UpdateNoise mtNoise;
  typedef Innovation mtInnovation;
  UpdateExample(){
    disablePreAndPostProcessingWarning_ = true;
  };
  virtual ~UpdateExample(){};
  void evalInnovation(mtInnovation& inn, const mtState& state, const mtNoise& noise) const{
    inn.get<Innovation::POS>() = state.get<State::ATT>().rotate(state.get<State::POS>())-meas_.get<UpdateMeas::POS>()+noise.get<UpdateNoise::POS>();
    inn.get<Innovation::ATT>() = (state.get<State::ATT>()*meas_.get<UpdateMeas::ATT>().inverted()).boxPlus(noise.get<UpdateNoise::ATT>());
  }
  void jacState(Eigen::MatrixXd& J, const mtState& state) const{
    mtInnovation inn;
    J.setZero();
    J.template block<3,3>(mtInnovation::getId<Innovation::POS>(),mtState::getId<State::POS>()) = MPD(state.get<State::ATT>()).matrix();
    J.template block<3,3>(mtInnovation::getId<Innovation::POS>(),mtState::getId<State::ATT>()) = -gSM(state.get<State::ATT>().rotate(state.get<State::POS>()));
    J.template block<3,3>(mtInnovation::getId<Innovation::ATT>(),mtState::getId<State::ATT>()) = M3D::Identity();
  }
  void jacNoise(Eigen::MatrixXd& J, const mtState& state) const{
    mtInnovation inn;
    J.setZero();
    J.template block<3,3>(mtInnovation::getId<Innovation::POS>(),mtNoise::getId<mtNoise::POS>()) = M3D::Identity();
    J.template block<3,3>(mtInnovation::getId<Innovation::ATT>(),mtNoise::getId<mtNoise::ATT>()) = M3D::Identity();
  }
};

class PredictionExample: public LWF::Prediction<FilterState>{
 public:
  using mtState = LWF::Prediction<FilterState>::mtState;
  typedef PredictionMeas mtMeas;
  typedef PredictionNoise mtNoise;
  PredictionExample(){
    disablePreAndPostProcessingWarning_ = true;
  };
  virtual ~PredictionExample(){};
  void evalPrediction(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
    V3D g_(0,0,-9.81);
    V3D dOmega = dt*(meas_.get<PredictionMeas::GYR>()-state.get<State::GYB>()-noise.get<PredictionNoise::ATT>()/sqrt(dt));
    QPD dQ = dQ.exponentialMap(dOmega);
    output.get<State::POS>() = state.get<State::POS>()+dt*state.get<State::ATT>().rotate(V3D(state.get<State::VEL>()+noise.get<PredictionNoise::POS>()/sqrt(dt)));
    output.get<State::VEL>() = (M3D::Identity()-gSM(dOmega))*state.get<State::VEL>()
        +dt*(meas_.get<PredictionMeas::ACC>()-state.get<State::ACB>()+state.get<State::ATT>().inverseRotate(g_)-noise.get<PredictionNoise::VEL>()/sqrt(dt));
    output.get<State::ACB>() = state.get<State::ACB>()+noise.get<PredictionNoise::ACB>()*sqrt(dt);
    output.get<State::GYB>() = state.get<State::GYB>()+noise.get<PredictionNoise::GYB>()*sqrt(dt);
    output.get<State::ATT>() = state.get<State::ATT>()*dQ;
    output.get<State::ATT>().fix();
  }
  void jacPreviousState(Eigen::MatrixXd& J, const mtState& state, double dt) const{
    V3D g_(0,0,-9.81);
    V3D dOmega = dt*(meas_.get<PredictionMeas::GYR>()-state.get<State::GYB>());
    J.setZero();
    J.template block<3,3>(mtState::getId<State::POS>(),mtState::getId<State::POS>()) = M3D::Identity();
    J.template block<3,3>(mtState::getId<State::POS>(),mtState::getId<State::VEL>()) = dt*MPD(state.get<State::ATT>()).matrix();
    J.template block<3,3>(mtState::getId<State::POS>(),mtState::getId<State::ATT>()) = -dt*gSM(V3D(state.get<State::ATT>().rotate(state.get<State::VEL>())));
    J.template block<3,3>(mtState::getId<State::VEL>(),mtState::getId<State::VEL>()) = (M3D::Identity()-gSM(dOmega));
    J.template block<3,3>(mtState::getId<State::VEL>(),mtState::getId<State::ACB>()) = -dt*M3D::Identity();
    J.template block<3,3>(mtState::getId<State::VEL>(),mtState::getId<State::GYB>()) = -dt*gSM(state.get<State::VEL>());
    J.template block<3,3>(mtState::getId<State::VEL>(),mtState::getId<State::ATT>()) = dt*MPD(state.get<State::ATT>()).matrix().transpose()*gSM(g_);
    J.template block<3,3>(mtState::getId<State::ACB>(),mtState::getId<State::ACB>()) = M3D::Identity();
    J.template block<3,3>(mtState::getId<State::GYB>(),mtState::getId<State::GYB>()) = M3D::Identity();
    J.template block<3,3>(mtState::getId<State::ATT>(),mtState::getId<State::GYB>()) = -dt*MPD(state.get<State::ATT>()).matrix()*Lmat(dOmega);
    J.template block<3,3>(mtState::getId<State::ATT>(),mtState::getId<State::ATT>()) = M3D::Identity();
  }
  void jacNoise(Eigen::MatrixXd& J, const mtState& state, double dt) const{
    mtNoise noise;
    V3D g_(0,0,-9.81);
    V3D dOmega = dt*(meas_.get<PredictionMeas::GYR>()-state.get<State::GYB>());
    J.setZero();
    J.template block<3,3>(mtState::getId<State::POS>(),mtNoise::getId<mtNoise::POS>()) = MPD(state.get<State::ATT>()).matrix()*sqrt(dt);
    J.template block<3,3>(mtState::getId<State::VEL>(),mtNoise::getId<mtNoise::VEL>()) = -M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::getId<State::VEL>(),mtNoise::getId<mtNoise::ATT>()) = -gSM(state.get<State::VEL>())*sqrt(dt);
    J.template block<3,3>(mtState::getId<State::ACB>(),mtNoise::getId<mtNoise::ACB>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::getId<State::GYB>(),mtNoise::getId<mtNoise::GYB>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::getId<State::ATT>(),mtNoise::getId<mtNoise::ATT>()) = -MPD(state.get<State::ATT>()).matrix()*Lmat(dOmega)*sqrt(dt);
  }
};

class PredictAndUpdateExample: public LWF::Update<Innovation,FilterState,UpdateMeas,UpdateNoise,OutlierDetectionExample,true>{
 public:
  using mtState = LWF::Update<Innovation,FilterState,UpdateMeas,UpdateNoise,OutlierDetectionExample,true>::mtState;
  typedef UpdateMeas mtMeas;
  typedef UpdateNoise mtNoise;
  typedef Innovation mtInnovation;
  PredictAndUpdateExample(){
    disablePreAndPostProcessingWarning_ = true;
  };
  virtual ~PredictAndUpdateExample(){};
  void evalInnovation(mtInnovation& inn, const mtState& state, const mtNoise& noise) const{
    inn.get<Innovation::POS>() = state.get<State::ATT>().rotate(state.get<State::POS>())-meas_.get<UpdateMeas::POS>()+noise.get<UpdateNoise::POS>();
    inn.get<Innovation::ATT>() = (state.get<State::ATT>()*meas_.get<UpdateMeas::ATT>().inverted()).boxPlus(noise.get<UpdateNoise::ATT>());
  }
  void jacState(Eigen::MatrixXd& J, const mtState& state) const{
    mtInnovation inn;
    J.setZero();
    J.template block<3,3>(mtInnovation::getId<Innovation::POS>(),mtState::getId<State::POS>()) = MPD(state.get<State::ATT>()).matrix();
    J.template block<3,3>(mtInnovation::getId<Innovation::POS>(),mtState::getId<State::ATT>()) = -gSM(state.get<State::ATT>().rotate(state.get<State::POS>()));
    J.template block<3,3>(mtInnovation::getId<Innovation::ATT>(),mtState::getId<State::ATT>()) = M3D::Identity();
  }
  void jacNoise(Eigen::MatrixXd& J, const mtState& state) const{
    mtInnovation inn;
    J.setZero();
    J.template block<3,3>(mtInnovation::getId<Innovation::POS>(),mtNoise::getId<mtNoise::POS>()) = M3D::Identity();
    J.template block<3,3>(mtInnovation::getId<Innovation::ATT>(),mtNoise::getId<mtNoise::ATT>()) = M3D::Identity();
  }
};

class GIFInnovation: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,5>>{
 public:
  enum StateNames {
    POS,
    VEL,
    ACB,
    GYB,
    ATT
  };
  GIFInnovation(){
    getName<0>() = "pos";
    getName<1>() = "vel";
    getName<2>() = "acb";
    getName<3>() = "gyb";
    getName<4>() = "att";
  }
  virtual ~GIFInnovation(){};
};
class GIFPredictionExample: public LWF::GIFPrediction<FilterState,GIFInnovation,PredictionMeas,PredictionNoise>{
 public:
  using mtState = FilterState::mtState;
  typedef PredictionMeas mtMeas;
  typedef PredictionNoise mtNoise;
  typedef GIFInnovation mtInnovation;
  V3D g_;
  GIFPredictionExample(): g_(0,0,-9.81){
    disablePreAndPostProcessingWarning_ = true;
  };
  virtual ~GIFPredictionExample(){};
  void evalResidual(mtInnovation& inn, const mtState& state0, const mtState& state1, const mtNoise& noise, double dt) const{
    V3D dOmega = dt*(meas_.get<PredictionMeas::GYR>()-state0.get<State::GYB>()-noise.get<PredictionNoise::ATT>()/sqrt(dt));
    inn.get<mtInnovation::POS>() = (state1.get<State::POS>() - state0.get<State::POS>())/dt - state0.get<State::ATT>().rotate(V3D(state0.get<State::VEL>() + noise.get<PredictionNoise::POS>()/sqrt(dt)));
    inn.get<mtInnovation::VEL>() = (state1.get<State::VEL>() - (M3D::Identity()-gSM(dOmega))*state0.get<State::VEL>())/dt
        - (meas_.get<PredictionMeas::ACC>()-state0.get<State::ACB>()+state0.get<State::ATT>().inverseRotate(g_) - noise.get<PredictionNoise::VEL>()/sqrt(dt));
    inn.get<mtInnovation::ACB>() = (state1.get<State::ACB>() - state0.get<State::ACB>())/dt + noise.get<PredictionNoise::ACB>()/sqrt(dt);
    inn.get<mtInnovation::GYB>() = (state1.get<State::GYB>() - state0.get<State::GYB>())/dt + noise.get<PredictionNoise::GYB>()/sqrt(dt);
    inn.get<mtInnovation::ATT>() = (state0.get<State::ATT>().inverted()*state1.get<State::ATT>()).logarithmicMap()/dt - dOmega/dt;
  }
  void jacPreviousState(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    V3D dOmega = -dt*(meas_.get<PredictionMeas::GYR>()-previousState.get<State::GYB>());
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtState::getId<State::POS>()) = -M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtState::getId<State::VEL>()) = -MPD(previousState.get<State::ATT>()).matrix();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtState::getId<State::ATT>()) = gSM(V3D(previousState.get<State::ATT>().rotate(V3D(previousState.get<State::VEL>()))));
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::VEL>()) = -(M3D::Identity()+gSM(dOmega))/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::ACB>()) = M3D::Identity();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::GYB>()) = gSM(previousState.get<State::VEL>());
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::ATT>()) = -MPD(previousState.get<State::ATT>()).matrix().transpose()*gSM(g_);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACB>(),mtState::getId<State::ACB>()) = -M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::GYB>(),mtState::getId<State::GYB>()) = -M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ATT>(),mtState::getId<State::GYB>()) = M3D::Identity();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ATT>(),mtState::getId<State::ATT>()) = -Lmat((previousState.get<State::ATT>().inverted()*currentState.get<State::ATT>()).logarithmicMap()).inverse()*MPD(previousState.get<State::ATT>().inverted()).matrix()/dt;
  }
  void jacCurrentState(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtState::getId<State::POS>()) = M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::VEL>()) = M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACB>(),mtState::getId<State::ACB>()) = M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::GYB>(),mtState::getId<State::GYB>()) = M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ATT>(),mtState::getId<State::ATT>()) = Lmat((previousState.get<State::ATT>().inverted()*currentState.get<State::ATT>()).logarithmicMap()).inverse()*MPD(previousState.get<State::ATT>().inverted()).matrix()/dt;
  }
  void jacNoise(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    V3D dOmega = -dt*(meas_.get<PredictionMeas::GYR>()-previousState.get<State::GYB>());
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtNoise::getId<mtNoise::POS>()) = -MPD(previousState.get<State::ATT>()).matrix()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtNoise::getId<mtNoise::VEL>()) = M3D::Identity()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtNoise::getId<mtNoise::ATT>()) = gSM(previousState.get<State::VEL>())/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACB>(),mtNoise::getId<mtNoise::ACB>()) = M3D::Identity()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::GYB>(),mtNoise::getId<mtNoise::GYB>()) = M3D::Identity()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ATT>(),mtNoise::getId<mtNoise::ATT>()) = M3D::Identity()/sqrt(dt);
  }
  void getLinearizationPoint(mtState& currentState, const mtFilterState& filterState, const mtMeas& meas, double dt){
    V3D dOmega = dt*(meas_.get<PredictionMeas::GYR>()-filterState.state_.get<State::GYB>());
    QPD dQ = dQ.exponentialMap(dOmega);
    currentState.get<State::POS>() = filterState.state_.get<State::POS>()+dt*filterState.state_.get<State::ATT>().rotate(filterState.state_.get<State::VEL>());
    currentState.get<State::VEL>() = (M3D::Identity()-gSM(dOmega))*filterState.state_.get<State::VEL>()
        +dt*(meas_.get<PredictionMeas::ACC>()-filterState.state_.get<State::ACB>()+filterState.state_.get<State::ATT>().inverseRotate(g_));
    currentState.get<State::ACB>() = filterState.state_.get<State::ACB>();
    currentState.get<State::GYB>() = filterState.state_.get<State::GYB>();
    currentState.get<State::ATT>() = filterState.state_.get<State::ATT>()*dQ;
    currentState.get<State::ATT>().fix();
  };
};

class GIFInnovationWithUpdate: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,6>,LWF::QuaternionElement>{
 public:
  enum StateNames {
    POS,
    VEL,
    ACB,
    GYB,
    ATT,
    POSU,
    ATTU
  };
  GIFInnovationWithUpdate(){
    getName<0>() = "pos";
    getName<1>() = "vel";
    getName<2>() = "acb";
    getName<3>() = "gyb";
    getName<4>() = "att";
    getName<5>() = "posu";
    getName<6>() = "attu";
  }
  virtual ~GIFInnovationWithUpdate(){};
};
class GIFNoiseWithUpdate: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,7>>{
 public:
  enum StateNames {
    POS,
    VEL,
    ACB,
    GYB,
    ATT,
    POSU,
    ATTU
  };
  GIFNoiseWithUpdate(){};
  virtual ~GIFNoiseWithUpdate(){};
};
class GIFMeasWithUpdate: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,3>,LWF::QuaternionElement>{
 public:
  enum StateNames {
    ACC,
    GYR,
    POS,
    ATT
  };
  GIFMeasWithUpdate(){};
  virtual ~GIFMeasWithUpdate(){};
};
class GIFPredictionExampleWithUpdate: public LWF::GIFPrediction<FilterState,GIFInnovationWithUpdate,GIFMeasWithUpdate,GIFNoiseWithUpdate>{
 public:
  using mtState = FilterState::mtState;
  typedef GIFMeasWithUpdate mtMeas;
  typedef GIFNoiseWithUpdate mtNoise;
  typedef GIFInnovationWithUpdate mtInnovation;
  V3D g_;
  GIFPredictionExampleWithUpdate(): g_(0,0,-9.81){
    disablePreAndPostProcessingWarning_ = true;
  };
  virtual ~GIFPredictionExampleWithUpdate(){};
  void evalResidual(mtInnovation& inn, const mtState& state0, const mtState& state1, const mtNoise& noise, double dt) const{
    V3D dOmega = dt*(meas_.get<PredictionMeas::GYR>()-state0.get<State::GYB>()-noise.get<PredictionNoise::ATT>()/sqrt(dt));
    inn.get<mtInnovation::POS>() = (state1.get<State::POS>() - state0.get<State::POS>())/dt - state0.get<State::ATT>().rotate(V3D(state0.get<State::VEL>() + noise.get<PredictionNoise::POS>()/sqrt(dt)));
    inn.get<mtInnovation::VEL>() = (state1.get<State::VEL>() - (M3D::Identity()-gSM(dOmega))*state0.get<State::VEL>())/dt
        - (meas_.get<PredictionMeas::ACC>()-state0.get<State::ACB>()+state0.get<State::ATT>().inverseRotate(g_) - noise.get<PredictionNoise::VEL>()/sqrt(dt));
    inn.get<mtInnovation::ACB>() = (state1.get<State::ACB>() - state0.get<State::ACB>())/dt + noise.get<PredictionNoise::ACB>()/sqrt(dt);
    inn.get<mtInnovation::GYB>() = (state1.get<State::GYB>() - state0.get<State::GYB>())/dt + noise.get<PredictionNoise::GYB>()/sqrt(dt);
    inn.get<mtInnovation::ATT>() = (state0.get<State::ATT>().inverted()*state1.get<State::ATT>()).logarithmicMap()/dt - dOmega/dt;
    inn.get<mtInnovation::POSU>() = state1.get<State::ATT>().rotate(state1.get<State::POS>())-meas_.get<mtMeas::POS>()+noise.get<mtNoise::POSU>();
    inn.get<mtInnovation::ATTU>() = (state1.get<State::ATT>()*meas_.get<mtMeas::ATT>().inverted()).boxPlus(noise.get<mtNoise::ATTU>());
  }
  void jacPreviousState(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    V3D dOmega = -dt*(meas_.get<PredictionMeas::GYR>()-previousState.get<State::GYB>());
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtState::getId<State::POS>()) = -M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtState::getId<State::VEL>()) = -MPD(previousState.get<State::ATT>()).matrix();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtState::getId<State::ATT>()) = gSM(V3D(previousState.get<State::ATT>().rotate(V3D(previousState.get<State::VEL>()))));
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::VEL>()) = -(M3D::Identity()+gSM(dOmega))/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::ACB>()) = M3D::Identity();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::GYB>()) = gSM(previousState.get<State::VEL>());
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::ATT>()) = -MPD(previousState.get<State::ATT>()).matrix().transpose()*gSM(g_);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACB>(),mtState::getId<State::ACB>()) = -M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::GYB>(),mtState::getId<State::GYB>()) = -M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ATT>(),mtState::getId<State::GYB>()) = M3D::Identity();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ATT>(),mtState::getId<State::ATT>()) = -Lmat((previousState.get<State::ATT>().inverted()*currentState.get<State::ATT>()).logarithmicMap()).inverse()*MPD(previousState.get<State::ATT>().inverted()).matrix()/dt;
  }
  void jacCurrentState(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtState::getId<State::POS>()) = M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::VEL>()) = M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACB>(),mtState::getId<State::ACB>()) = M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::GYB>(),mtState::getId<State::GYB>()) = M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ATT>(),mtState::getId<State::ATT>()) = Lmat((previousState.get<State::ATT>().inverted()*currentState.get<State::ATT>()).logarithmicMap()).inverse()*MPD(previousState.get<State::ATT>().inverted()).matrix()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POSU>(),mtState::getId<State::POS>()) = MPD(currentState.get<State::ATT>()).matrix();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POSU>(),mtState::getId<State::ATT>()) = -gSM(currentState.get<State::ATT>().rotate(currentState.get<State::POS>()));
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ATTU>(),mtState::getId<State::ATT>()) = M3D::Identity();
  }
  void jacNoise(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    V3D dOmega = -dt*(meas_.get<PredictionMeas::GYR>()-previousState.get<State::GYB>());
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtNoise::getId<mtNoise::POS>()) = -MPD(previousState.get<State::ATT>()).matrix()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtNoise::getId<mtNoise::VEL>()) = M3D::Identity()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtNoise::getId<mtNoise::ATT>()) = gSM(previousState.get<State::VEL>())/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACB>(),mtNoise::getId<mtNoise::ACB>()) = M3D::Identity()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::GYB>(),mtNoise::getId<mtNoise::GYB>()) = M3D::Identity()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ATT>(),mtNoise::getId<mtNoise::ATT>()) = M3D::Identity()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POSU>(),mtNoise::getId<mtNoise::POSU>()) = M3D::Identity();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ATTU>(),mtNoise::getId<mtNoise::ATTU>()) = M3D::Identity();
  }
  void getLinearizationPoint(mtState& currentState, const mtFilterState& filterState, const mtMeas& meas, double dt){
    V3D dOmega = dt*(meas_.get<PredictionMeas::GYR>()-filterState.state_.get<State::GYB>());
    QPD dQ = dQ.exponentialMap(dOmega);
    currentState.get<State::POS>() = filterState.state_.get<State::POS>()+dt*filterState.state_.get<State::ATT>().rotate(filterState.state_.get<State::VEL>());
    currentState.get<State::VEL>() = (M3D::Identity()-gSM(dOmega))*filterState.state_.get<State::VEL>()
        +dt*(meas_.get<PredictionMeas::ACC>()-filterState.state_.get<State::ACB>()+filterState.state_.get<State::ATT>().inverseRotate(g_));
    currentState.get<State::ACB>() = filterState.state_.get<State::ACB>();
    currentState.get<State::GYB>() = filterState.state_.get<State::GYB>();
    currentState.get<State::ATT>() = filterState.state_.get<State::ATT>()*dQ;
    currentState.get<State::ATT>().fix();
  }
};

}

namespace Linear{

class State: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,2>>{
 public:
  enum StateNames {
    POS,
    VEL
  };
  State(){
    createDefaultNames("s");
  };
  virtual ~State(){};
};
class UpdateMeas: public LWF::State<LWF::ScalarElement,LWF::VectorElement<3>>{
 public:
  enum StateNames {
    HEI,
    POS
  };
  UpdateMeas(){};
  virtual ~UpdateMeas(){};
};
class UpdateNoise: public LWF::State<LWF::ScalarElement,LWF::VectorElement<3>>{
 public:
  enum StateNames {
    HEI,
    POS
  };
  UpdateNoise(){};
  virtual ~UpdateNoise(){};
};
class Innovation: public LWF::State<LWF::ScalarElement,LWF::VectorElement<3>>{
 public:
  enum StateNames {
    HEI,
    POS
  };
  Innovation(){};
  virtual ~Innovation(){};
};
class PredictionNoise: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{
 public:
  enum StateNames {
    VEL,
    ACC
  };
  PredictionNoise(){};
  virtual ~PredictionNoise(){};
};
class PredictionMeas: public LWF::State<LWF::VectorElement<3>>{
 public:
  enum StateNames {
    ACC
  };
  PredictionMeas(){};
  virtual ~PredictionMeas(){};
};
class OutlierDetectionExample: public LWF::OutlierDetection<LWF::ODEntry<0,3,1>>{
 public:
  virtual ~OutlierDetectionExample(){};
};
class FilterState: public LWF::FilterState<State,PredictionMeas,PredictionNoise,UpdateNoise::D_>{
 public:
  virtual ~FilterState(){};
};

class UpdateExample: public LWF::Update<Innovation,FilterState,UpdateMeas,UpdateNoise,OutlierDetectionExample,false>{
 public:
  using mtState = LWF::Update<Innovation,FilterState,UpdateMeas,UpdateNoise,OutlierDetectionExample,false>::mtState;
  typedef UpdateMeas mtMeas;
  typedef UpdateNoise mtNoise;
  typedef Innovation mtInnovation;
  UpdateExample(){
    disablePreAndPostProcessingWarning_ = true;
  };
  virtual ~UpdateExample(){};
  void evalInnovation(mtInnovation& inn, const mtState& state, const mtNoise& noise) const{
    inn.get<Innovation::POS>() = state.get<State::POS>()-meas_.get<UpdateMeas::POS>()+noise.get<UpdateNoise::POS>();
    inn.get<Innovation::HEI>() = V3D(0,0,1).dot(state.get<State::POS>())-meas_.get<UpdateMeas::HEI>()+noise.get<UpdateNoise::HEI>();
  }
  void jacState(Eigen::MatrixXd& J, const mtState& state) const{
    mtInnovation inn;
    J.setZero();
    J.template block<3,3>(mtInnovation::getId<Innovation::POS>(),mtState::getId<State::POS>()) = M3D::Identity();
    J.template block<1,3>(mtInnovation::getId<Innovation::HEI>(),mtState::getId<State::POS>()) = V3D(0,0,1).transpose();
  }
  void jacNoise(Eigen::MatrixXd& J, const mtState& state) const{
    mtInnovation inn;
    J.setZero();
    J.template block<3,3>(mtInnovation::getId<Innovation::POS>(),mtNoise::getId<mtNoise::POS>()) = M3D::Identity();
    J(mtInnovation::getId<Innovation::HEI>(),mtNoise::getId<mtNoise::HEI>()) = 1.0;
  }
};

class PredictionExample: public LWF::Prediction<FilterState>{
 public:
  using mtState = LWF::Prediction<FilterState>::mtState;
  typedef PredictionMeas mtMeas;
  typedef PredictionNoise mtNoise;
  PredictionExample(){
    disablePreAndPostProcessingWarning_ = true;
  };
  virtual ~PredictionExample(){};
  void evalPrediction(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
    output.get<mtState::POS>() = state.get<mtState::POS>()+dt*state.get<mtState::VEL>()+noise.get<PredictionNoise::VEL>()*sqrt(dt);
    output.get<mtState::VEL>() = state.get<mtState::VEL>()+dt*meas_.get<mtMeas::ACC>()+noise.get<PredictionNoise::ACC>()*sqrt(dt);
  }
  void jacPreviousState(Eigen::MatrixXd& J, const mtState& state, double dt) const{
    J.setZero();
    J.template block<3,3>(mtState::getId<State::POS>(),mtState::getId<State::POS>()) = M3D::Identity();
    J.template block<3,3>(mtState::getId<State::POS>(),mtState::getId<State::VEL>()) = dt*M3D::Identity();
    J.template block<3,3>(mtState::getId<State::VEL>(),mtState::getId<State::VEL>()) = M3D::Identity();
  }
  void jacNoise(Eigen::MatrixXd& J, const mtState& state, double dt) const{
    J.setZero();
    J.template block<3,3>(mtState::getId<State::POS>(),mtNoise::getId<mtNoise::VEL>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::getId<State::VEL>(),mtNoise::getId<mtNoise::ACC>()) = M3D::Identity()*sqrt(dt);
  }
};

class PredictAndUpdateExample: public LWF::Update<Innovation,FilterState,UpdateMeas,UpdateNoise,OutlierDetectionExample,true>{
 public:
  using mtState = LWF::Update<Innovation,FilterState,UpdateMeas,UpdateNoise,OutlierDetectionExample,true>::mtState;
  typedef UpdateMeas mtMeas;
  typedef UpdateNoise mtNoise;
  typedef Innovation mtInnovation;
  PredictAndUpdateExample(){
    disablePreAndPostProcessingWarning_ = true;
  };
  virtual ~PredictAndUpdateExample(){};
  void evalInnovation(mtInnovation& inn, const mtState& state, const mtNoise& noise) const{
    inn.get<Innovation::POS>() = state.get<State::POS>()-meas_.get<UpdateMeas::POS>()+noise.get<UpdateNoise::POS>();
    inn.get<Innovation::HEI>() = V3D(0,0,1).dot(state.get<State::POS>())-meas_.get<UpdateMeas::HEI>()+noise.get<UpdateNoise::HEI>();
  }
  void jacState(Eigen::MatrixXd& J, const mtState& state) const{
    mtInnovation inn;
    J.setZero();
    J.template block<3,3>(mtInnovation::getId<Innovation::POS>(),mtState::getId<State::POS>()) = M3D::Identity();
    J.template block<1,3>(mtInnovation::getId<Innovation::HEI>(),mtState::getId<State::POS>()) = V3D(0,0,1).transpose();
  }
  void jacNoise(Eigen::MatrixXd& J, const mtState& state) const{
    mtInnovation inn;
    J.setZero();
    J.template block<3,3>(mtInnovation::getId<Innovation::POS>(),mtNoise::getId<mtNoise::POS>()) = M3D::Identity();
    J(mtInnovation::getId<Innovation::HEI>(),mtNoise::getId<mtNoise::HEI>()) = 1.0;
  }
};

class GIFInnovation: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{
 public:
  enum StateNames {
    VEL,
    ACC
  };
  GIFInnovation(){};
  virtual ~GIFInnovation(){};
};
class GIFPredictionExample: public LWF::GIFPrediction<FilterState,GIFInnovation,PredictionMeas,PredictionNoise>{
 public:
  using mtState = FilterState::mtState;
  typedef PredictionMeas mtMeas;
  typedef PredictionNoise mtNoise;
  typedef GIFInnovation mtInnovation;
  GIFPredictionExample(){
    disablePreAndPostProcessingWarning_ = true;
  };
  virtual ~GIFPredictionExample(){};
  void evalResidual(mtInnovation& inn, const mtState& state0, const mtState& state1, const mtNoise& noise, double dt) const{
    inn.get<mtInnovation::VEL>() = (state1.get<mtState::POS>() - state0.get<mtState::POS>())/dt - state0.get<mtState::VEL>() + noise.get<PredictionNoise::VEL>()/sqrt(dt);
    inn.get<mtInnovation::ACC>() = (state1.get<mtState::VEL>() - state0.get<mtState::VEL>())/dt - meas_.get<mtMeas::ACC>() + noise.get<PredictionNoise::ACC>()/sqrt(dt);
  }
  void jacPreviousState(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::POS>()) = -M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::VEL>()) = -M3D::Identity();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACC>(),mtState::getId<State::VEL>()) = -M3D::Identity()/dt;
  }
  void jacCurrentState(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::POS>()) = M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACC>(),mtState::getId<State::VEL>()) = M3D::Identity()/dt;
  }
  void jacNoise(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtNoise::getId<mtNoise::VEL>()) = M3D::Identity()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACC>(),mtNoise::getId<mtNoise::ACC>()) = M3D::Identity()/sqrt(dt);
  }
};

class GIFInnovationWithUpdate: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>,LWF::VectorElement<3>,LWF::ScalarElement>{
 public:
  enum StateNames {
    VEL,
    ACC,
    POS,
    HEI
  };
  GIFInnovationWithUpdate(){};
  virtual ~GIFInnovationWithUpdate(){};
};
class GIFNoiseWithUpdate: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>,LWF::VectorElement<3>,LWF::ScalarElement>{
 public:
  enum StateNames {
    VEL,
    ACC,
    POS,
    HEI
  };
  GIFNoiseWithUpdate(){};
  virtual ~GIFNoiseWithUpdate(){};
};
class GIFMeasWithUpdate: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>,LWF::ScalarElement>{
 public:
  enum StateNames {
    ACC,
    POS,
    HEI
  };
  GIFMeasWithUpdate(){};
  virtual ~GIFMeasWithUpdate(){};
};
class GIFPredictionExampleWithUpdate: public LWF::GIFPrediction<FilterState,GIFInnovationWithUpdate,GIFMeasWithUpdate,GIFNoiseWithUpdate>{
 public:
  using mtState = FilterState::mtState;
  typedef GIFMeasWithUpdate mtMeas;
  typedef GIFNoiseWithUpdate mtNoise;
  typedef GIFInnovationWithUpdate mtInnovation;
  GIFPredictionExampleWithUpdate(){
    disablePreAndPostProcessingWarning_ = true;
  };
  virtual ~GIFPredictionExampleWithUpdate(){};
  void evalResidual(mtInnovation& inn, const mtState& state0, const mtState& state1, const mtNoise& noise, double dt) const{
    inn.get<mtInnovation::VEL>() = (state1.get<mtState::POS>() - state0.get<mtState::POS>())/dt - state0.get<mtState::VEL>() + noise.get<mtNoise::VEL>()/sqrt(dt);
    inn.get<mtInnovation::ACC>() = (state1.get<mtState::VEL>() - state0.get<mtState::VEL>())/dt - meas_.get<mtMeas::ACC>() + noise.get<mtNoise::ACC>()/sqrt(dt);
    inn.get<mtInnovation::POS>() = state1.get<State::POS>()-meas_.get<mtMeas::POS>()+noise.get<mtNoise::POS>();
    inn.get<mtInnovation::HEI>() = V3D(0,0,1).dot(state1.get<State::POS>())-meas_.get<mtMeas::HEI>()+noise.get<mtNoise::HEI>();
  }
  void jacPreviousState(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::POS>()) = -M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::VEL>()) = -M3D::Identity();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACC>(),mtState::getId<State::VEL>()) = -M3D::Identity()/dt;
  }
  void jacCurrentState(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtState::getId<State::POS>()) = M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACC>(),mtState::getId<State::VEL>()) = M3D::Identity()/dt;
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtState::getId<State::POS>()) = M3D::Identity();
    F.template block<1,3>(mtInnovation::getId<mtInnovation::HEI>(),mtState::getId<State::POS>()) = V3D(0,0,1).transpose();
  }
  void jacNoise(Eigen::MatrixXd& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setZero();
    F.template block<3,3>(mtInnovation::getId<mtInnovation::VEL>(),mtNoise::getId<mtNoise::VEL>()) = M3D::Identity()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::ACC>(),mtNoise::getId<mtNoise::ACC>()) = M3D::Identity()/sqrt(dt);
    F.template block<3,3>(mtInnovation::getId<mtInnovation::POS>(),mtNoise::getId<mtNoise::POS>()) = M3D::Identity();
    F(mtInnovation::getId<mtInnovation::HEI>(),mtNoise::getId<mtNoise::HEI>()) = 1.0;
  }
};

}

class NonlinearTest{
 public:
  static const int id_ = 0;
  typedef Nonlinear::State mtState;
  typedef Nonlinear::FilterState mtFilterState;
  typedef Nonlinear::UpdateMeas mtUpdateMeas;
  typedef Nonlinear::UpdateNoise mtUpdateNoise;
  typedef Nonlinear::Innovation mtInnovation;
  typedef Nonlinear::PredictionNoise mtPredictionNoise;
  typedef Nonlinear::PredictionMeas mtPredictionMeas;
  typedef Nonlinear::UpdateExample mtUpdateExample;
  typedef Nonlinear::PredictionExample mtPredictionExample;
  typedef Nonlinear::PredictAndUpdateExample mtPredictAndUpdateExample;
  typedef Nonlinear::OutlierDetectionExample mtOutlierDetectionExample;
  typedef Nonlinear::GIFPredictionExample mtGIFPredictionExample;
  typedef Nonlinear::GIFPredictionExampleWithUpdate mtGIFPredictionExampleWithUpdate;
  typedef Nonlinear::GIFInnovationWithUpdate mtGIFInnovationWithUpdate;
  typedef Nonlinear::GIFNoiseWithUpdate mtGIFNoiseWithUpdate;
  typedef Nonlinear::GIFMeasWithUpdate mtGIFMeasWithUpdate;
  void init(mtState& state,mtUpdateMeas& updateMeas,mtPredictionMeas& predictionMeas){
    state.get<mtState::POS>() = V3D(2.1,-0.2,-1.9);
    state.get<mtState::VEL>() = V3D(0.3,10.9,2.3);
    state.get<mtState::ACB>() = V3D(0.3,10.9,2.3);
    state.get<mtState::GYB>() = V3D(0.3,10.9,2.3);
    state.get<mtState::ATT>() = QPD(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));
    updateMeas.get<mtUpdateMeas::POS>() = V3D(-1.5,12,5.23);
    updateMeas.get<mtUpdateMeas::ATT>() = QPD(3.0/sqrt(15.0),-1.0/sqrt(15.0),1.0/sqrt(15.0),2.0/sqrt(15.0));
    predictionMeas.get<mtPredictionMeas::ACC>() = V3D(-5,2,17.3);
    predictionMeas.get<mtPredictionMeas::GYR>() = V3D(15.7,0.45,-2.3);
  }
  static void mergePredictionAndUpdateMeas(mtGIFMeasWithUpdate& merged, mtPredictionMeas& predictionMeas, mtUpdateMeas& updateMeas){
    merged.template get<mtGIFMeasWithUpdate::ACC>() = predictionMeas.template get<mtPredictionMeas::ACC>();
    merged.template get<mtGIFMeasWithUpdate::GYR>() = predictionMeas.template get<mtPredictionMeas::GYR>();
    merged.template get<mtGIFMeasWithUpdate::POS>() = updateMeas.template get<mtUpdateMeas::POS>();
    merged.template get<mtGIFMeasWithUpdate::ATT>() = updateMeas.template get<mtUpdateMeas::ATT>();
  }
};

class LinearTest{
 public:
  static const int id_ = 1;
  typedef Linear::State mtState;
  typedef Linear::FilterState mtFilterState;
  typedef Linear::UpdateMeas mtUpdateMeas;
  typedef Linear::UpdateNoise mtUpdateNoise;
  typedef Linear::Innovation mtInnovation;
  typedef Linear::PredictionNoise mtPredictionNoise;
  typedef Linear::PredictionMeas mtPredictionMeas;
  typedef Linear::UpdateExample mtUpdateExample;
  typedef Linear::PredictionExample mtPredictionExample;
  typedef Linear::PredictAndUpdateExample mtPredictAndUpdateExample;
  typedef Linear::OutlierDetectionExample mtOutlierDetectionExample;
  typedef Linear::GIFPredictionExample mtGIFPredictionExample;
  typedef Linear::GIFPredictionExampleWithUpdate mtGIFPredictionExampleWithUpdate;
  typedef Linear::GIFInnovationWithUpdate mtGIFInnovationWithUpdate;
  typedef Linear::GIFNoiseWithUpdate mtGIFNoiseWithUpdate;
  typedef Linear::GIFMeasWithUpdate mtGIFMeasWithUpdate;
  void init(mtState& state,mtUpdateMeas& updateMeas,mtPredictionMeas& predictionMeas){
    state.get<mtState::POS>() = V3D(2.1,-0.2,-1.9);
    state.get<mtState::VEL>() = V3D(0.3,10.9,2.3);
    updateMeas.get<mtUpdateMeas::POS>() = V3D(-1.5,12,5.23);
    updateMeas.get<mtUpdateMeas::HEI>() = 0.5;
    predictionMeas.get<mtPredictionMeas::ACC>() = V3D(-5,2,17.3);
  }
  static void mergePredictionAndUpdateMeas(mtGIFMeasWithUpdate& merged, mtPredictionMeas& predictionMeas, mtUpdateMeas& updateMeas){
    merged.template get<mtGIFMeasWithUpdate::ACC>() = predictionMeas.template get<mtPredictionMeas::ACC>();
    merged.template get<mtGIFMeasWithUpdate::POS>() = updateMeas.template get<mtUpdateMeas::POS>();
    merged.template get<mtGIFMeasWithUpdate::HEI>() = updateMeas.template get<mtUpdateMeas::HEI>();
  }
};

}

#endif /* LWF_TestClasses_HPP_ */
