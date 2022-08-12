#include <iostream>
#include "BundleAdjustmentByG2O.hpp"
#include "BAg2o.hpp"
int main() {
//  BundleAdjustmentByG2O ba;
//  ba.testTcw();
//  std::cout << "===============================================================" << std::endl;
//  BundleAdjustmentByG2O ba2;
//  ba2.testTcwOnlyPose();
  std::string solverType;
  std::string robustType;
  BAG2O ba(solverType,robustType);
  ba.test(0.1,2,false);
  return 0;
}