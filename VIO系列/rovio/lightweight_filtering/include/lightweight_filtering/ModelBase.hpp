/*
 * ModelBase.hpp
 *
 *  Created on: Feb 9, 2014
 *      Author: Bloeschm
 */

#ifndef LWF_ModelBase_HPP_
#define LWF_ModelBase_HPP_

#include "lightweight_filtering/common.hpp"

namespace LWF{

template<typename DERIVED, typename Output, typename... Inputs>
class ModelBase{
 public:
  static const unsigned int nInputs_ = sizeof...(Inputs);
  typedef Output mtOutput;
  typedef std::tuple<Inputs...> mtInputTuple;
  ModelBase(){};
  virtual ~ModelBase(){};
  void eval(mtOutput& output, const mtInputTuple& inputs, double dt = 0.0) const{
    static_cast<const DERIVED&>(*this).eval_(output,inputs,dt);
  }
  template<int i>
  void jacInput(Eigen::MatrixXd& F, const mtInputTuple& inputs, double dt = 0.0) const{
    static_cast<const DERIVED&>(*this).template jacInput_<i>(F,inputs,dt);
  }
  template<int i,int s = 0, int n = std::tuple_element<i,mtInputTuple>::type::D_>
  void jacInputFD(Eigen::MatrixXd& F, const mtInputTuple& inputs, double dt, double d) const{
    static_assert(s + n <= (std::tuple_element<i,mtInputTuple>::type::D_), "Bad dimension for evaluating jacInputFD");
    mtInputTuple inputsDisturbed = inputs;
    typename std::tuple_element<i,mtInputTuple>::type::mtDifVec difVec;
    mtOutput outputReference;
    mtOutput outputDisturbed;
    eval(outputReference,inputs,dt);
    typename mtOutput::mtDifVec dif;
    for(unsigned int j=s;j<s+n;j++){
      difVec.setZero();
      difVec(j) = d;
      std::get<i>(inputs).boxPlus(difVec,std::get<i>(inputsDisturbed));
      eval(outputDisturbed,inputsDisturbed,dt);
      outputDisturbed.boxMinus(outputReference,dif);
      F.col(j) = dif/d;
    }
  }
  template<int i>
  bool testJacInput(double d = 1e-6,double th = 1e-6,unsigned int s = 0,double dt = 0.1) const{
    mtInputTuple inputs;
    setRandomInputs(inputs,s);
    return testJacInput<i>(inputs,d,th,dt);
  }
  template<int i>
  bool testJacInput(const mtInputTuple& inputs, double d = 1e-6,double th = 1e-6,double dt = 0.1) const{
    Eigen::MatrixXd F((int)(mtOutput::D_),(int)(std::tuple_element<i,mtInputTuple>::type::D_));
    Eigen::MatrixXd F_FD((int)(mtOutput::D_),(int)(std::tuple_element<i,mtInputTuple>::type::D_));
    mtOutput output;
    typename Eigen::MatrixXd::Index maxRow, maxCol = 0;
    jacInput<i>(F,inputs,dt);
    jacInputFD<i>(F_FD,inputs,dt,d);
    const double r = (F-F_FD).array().abs().maxCoeff(&maxRow, &maxCol);
    if(r>th){
      unsigned int outputId = mtOutput::getElementId(maxRow);
      unsigned int inputId = std::tuple_element<i,mtInputTuple>::type::getElementId(maxCol);
      std::cout << "==== Model jacInput Test failed: " << r << " is larger than " << th << " at row "
          << maxRow << "("<< output.getName(outputId) << ") and col " << maxCol << "("<< std::get<i>(inputs).getName(inputId) << ") ====" << std::endl;
      std::cout << "  " << F(maxRow,maxCol) << "  " << F_FD(maxRow,maxCol) << std::endl;
      return false;
    } else {
      std::cout << "==== Test successful (" << r << ") ====" << std::endl;
      return true;
    }
  }
  static inline void setRandomInputs(mtInputTuple& inputs,unsigned int& s){
    _setRandomInputs(inputs,s);
  }
  template<int i=0,typename std::enable_if<(i<nInputs_)>::type* = nullptr>
  static inline void _setRandomInputs(mtInputTuple& inputs,unsigned int& s){
    std::get<i>(inputs).setRandom(s);
    _setRandomInputs<i+1>(inputs,s);
  }
  template<int i=0,typename std::enable_if<(i>=nInputs_)>::type* = nullptr>
  static inline void _setRandomInputs(mtInputTuple& inputs,unsigned int& s){}
  bool testJacs(unsigned int& s, double d = 1e-6,double th = 1e-6,double dt = 0.1) const{
    mtInputTuple inputs;
    setRandomInputs(inputs,s);
    return testJacs(inputs,d,th,dt);
  }
  bool testJacs(double d = 1e-6,double th = 1e-6,double dt = 0.1) const{
    mtInputTuple inputs;
    unsigned int s = 1;
    setRandomInputs(inputs,s);
    return testJacs(inputs,d,th,dt);
  }
  bool testJacs(const mtInputTuple& inputs, double d = 1e-6,double th = 1e-6,double dt = 0.1) const{
    return _testJacs(inputs,d,th,dt);
  }
  template<int i=0,typename std::enable_if<(i<nInputs_)>::type* = nullptr>
  bool _testJacs(const mtInputTuple& inputs, double d = 1e-6,double th = 1e-6,double dt = 0.1) const{
    return (testJacInput<i>(inputs,d,th,dt) & _testJacs<i+1>(inputs,d,th,dt));
  }
  template<int i=0,typename std::enable_if<(i>=nInputs_)>::type* = nullptr>
  bool _testJacs(const mtInputTuple& inputs, double d = 1e-6,double th = 1e-6,double dt = 0.1) const{
    return true;
  }
};

}

#endif /* LWF_ModelBase_HPP_ */
