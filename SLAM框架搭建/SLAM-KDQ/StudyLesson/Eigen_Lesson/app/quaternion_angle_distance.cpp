//
// Created by kdq on 2021/7/2.
//
#include "iostream"
#include "Eigen/Dense"

int main() {

  Eigen::AngleAxisf axis1(M_PI,Eigen::Vector3f(0,0,1));
  Eigen::AngleAxisf axis2(M_PI/4,Eigen::Vector3f(0,0,1));
  Eigen::Quaternionf  q1(axis1);
  Eigen::Quaternionf  q2(axis2);
  //返回两个四元数之间的角度距离
  std::cout << "distance = " <<  q1.angularDistance(q2) * 180 / M_PI << std::endl;
  return 0;
}

