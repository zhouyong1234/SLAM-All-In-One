/*
 * PropertyHandler.hpp
 *
 *  Created on: Oct 27, 2014
 *      Author: Bloeschm
 */

#ifndef LWF_PropertyHandler_HPP_
#define LWF_PropertyHandler_HPP_

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include "lightweight_filtering/common.hpp"
#include <unordered_map>
#include <unordered_set>

namespace rot = kindr;

namespace LWF{

template<typename TYPE>
class Register{
 public:
  typedef boost::property_tree::ptree ptree;
  Register(){};
  virtual ~Register(){};
  std::map<TYPE*,std::string> registerMap_;
  std::unordered_set<TYPE*> zeros_;
  void registerZero(TYPE& var){
    if(zeros_.count(&var)!=0) std::cout << "Property Handler Error: Zero variable already registered." << std::endl;
    zeros_.insert(&var);
  }
  void registerScalar(std::string str, TYPE& var){
    if(registerMap_.count(&var)!=0) std::cout << "Property Handler Error: Variable already registered to " << str << "." << std::endl;
    registerMap_[&var] = str;
  }
  void registerVector(std::string str, Eigen::Matrix<TYPE,3,1>& var){
    registerScalar(str + "_x",var(0));
    registerScalar(str + "_y",var(1));
    registerScalar(str + "_z",var(2));
  }
  void registerQuaternion(std::string str, rot::RotationQuaternion<TYPE>& var){
    registerScalar(str + "_w",var.toImplementation().w());
    registerScalar(str + "_x",var.toImplementation().x());
    registerScalar(str + "_y",var.toImplementation().y());
    registerScalar(str + "_z",var.toImplementation().z());
  }
  template<int N, int M>
  void registerMatrix(std::string str, Eigen::Matrix<TYPE,N,M>& var){
    for(unsigned int i=0;i<N;i++){
      for(unsigned int j=0;j<M;j++){
        registerScalar(str + "_" + std::to_string(i) + "_" + std::to_string(j),var(i,j));
      }
    }
  }
  template <typename Derived>
  void registerDiagonalMatrix(std::string str, Eigen::MatrixBase<Derived>& var){
    const int N = var.rows();
    for(unsigned int i=0;i<N;i++){
      registerScalar(str + "_" + std::to_string(i),var(i,i));
    }
    for(unsigned int i=0;i<N;i++){
      for(unsigned int j=0;j<N;j++){
        if(i!=j) registerZero(var(i,j));
      }
    }
  }
  template<int N>
  void registerScaledUnitMatrix(std::string str, Eigen::Matrix<TYPE,N,N>& var){
    for(unsigned int i=0;i<N;i++){
      registerScalar(str,var(i,i));
    }
    for(unsigned int i=0;i<N;i++){
      for(unsigned int j=0;j<N;j++){
        if(i!=j) registerZero(var(i,j));
      }
    }
  }
  void removeScalarByStr(std::string str){ // slow
    bool found = false;
    for(auto it=registerMap_.begin(); it != registerMap_.end(); ++it){
      if(it->second == str){
        registerMap_.erase(it);
        found = true;
        break;
      }
    }
    if(!found) std::cout << "Property Handler Error: Cannot remove variable with str = " << str << std::endl;
  }
  void removeScalarByVar(TYPE& var){
    if(registerMap_.count(&var)==0) std::cout << "Property Handler Error: Cannot remove variable, does not exist!" << std::endl;
    registerMap_.erase(&var);
  }
  void buildPropertyTree(ptree& pt){
    for(typename std::map<TYPE*,std::string>::iterator it=registerMap_.begin(); it != registerMap_.end(); ++it){
      pt.put(it->second, *(it->first));
    }
  }
  void readPropertyTree(ptree& pt){
    for(typename std::map<TYPE*,std::string>::iterator it=registerMap_.begin(); it != registerMap_.end(); ++it){
      *(it->first) = pt.get<TYPE>(it->second);
    }
    for(typename std::unordered_set<TYPE*>::iterator it=zeros_.begin(); it != zeros_.end(); ++it){
      **it = static_cast<TYPE>(0);
    }
  }
};

class PropertyHandler{
 public:
  typedef boost::property_tree::ptree ptree;
  PropertyHandler(){};
  virtual ~PropertyHandler(){};
  Register<bool> boolRegister_;
  Register<int> intRegister_;
  Register<double> doubleRegister_;
  Register<std::string> stringRegister_;
  std::unordered_map<std::string,PropertyHandler*> subHandlers_;
  void buildPropertyTree(ptree& pt){
    boolRegister_.buildPropertyTree(pt);
    intRegister_.buildPropertyTree(pt);
    doubleRegister_.buildPropertyTree(pt);
    stringRegister_.buildPropertyTree(pt);
    for(typename std::unordered_map<std::string,PropertyHandler*>::iterator it=subHandlers_.begin(); it != subHandlers_.end(); ++it){
      ptree ptsub;
      it->second->buildPropertyTree(ptsub);
      pt.add_child(it->first,ptsub);
    }
  }
  void readPropertyTree(ptree& pt){
    boolRegister_.readPropertyTree(pt);
    intRegister_.readPropertyTree(pt);
    doubleRegister_.readPropertyTree(pt);
    stringRegister_.readPropertyTree(pt);
    for(typename std::unordered_map<std::string,PropertyHandler*>::iterator it=subHandlers_.begin(); it != subHandlers_.end(); ++it){
      ptree ptsub;
      ptsub = pt.get_child(it->first);
      it->second->readPropertyTree(ptsub);
    }
  }
  void registerSubHandler(std::string str,PropertyHandler& subHandler){
    if(subHandlers_.count(str)!=0) std::cout << "Property Handler Error: subHandler with name " << str << " already exists" << std::endl;
    subHandlers_[str] = &subHandler;
  }
  void writeToInfo(const std::string &filename){
    ptree pt;
    buildPropertyTree(pt);
    write_info(filename,pt);
  }
  void readFromInfo(const std::string &filename){
    ptree ptDefault;
    buildPropertyTree(ptDefault);
    ptree pt;
    try{
      read_info(filename,pt);
      readPropertyTree(pt);
      refreshProperties();
      for(typename std::unordered_map<std::string,PropertyHandler*>::iterator it=subHandlers_.begin(); it != subHandlers_.end(); ++it){
        it->second->refreshProperties();
      }
    } catch (boost::property_tree::ptree_error& e){
      std::cout << "An exception occurred. " << e.what() << std::endl;
      std::string newFilename = filename + "_new";
      std::cout << "\033[31mWriting a new info-file to " << newFilename << "\033[0m" << std::endl;
      write_info(newFilename,ptDefault);
      refreshProperties();
      for(typename std::unordered_map<std::string,PropertyHandler*>::iterator it=subHandlers_.begin(); it != subHandlers_.end(); ++it){
        it->second->refreshProperties();
      }
    }
  }
  virtual void refreshProperties(){};
};

}

#endif /* LWF_PropertyHandler_HPP_ */
