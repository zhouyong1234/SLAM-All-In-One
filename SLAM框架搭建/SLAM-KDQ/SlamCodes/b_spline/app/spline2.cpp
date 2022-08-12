//
// Created by kdq on 2021/6/17.
//
#include <fstream>
#include "BSplineX.hpp"
Eigen::Vector3d f(double x) {
  Eigen::Vector3d y;
  y.x() = x * x - 2 * x;
  y.y() = cos(x);
  y.z() = sin(x);
  return y;
}

int main() {
  const int D = 3;
  const int S = 20;
  const int K = 5;
  Eigen::Matrix<double,S,1> x;
  Eigen::Matrix<double,S,D> y;
  int j = 0;
  for (int i = 10; i < 10 + S; i++) {
    Eigen::Vector3d sy = f(i);
    x(j,0) = i;
    y.row(j) = sy;
    j++;
  }
  std::ofstream file("spline2.csv",std::ios::out);
  file << "t,yx,yy,yz,eyx,eyy,eyz,evx,evy,evz,eax,eay,eaz" << std::endl;
  BSplineX<S,D,K> splineX(x,y);
  for (double i = 10; i < 10 + S; i += 0.1) {
    Eigen::Vector3d real = f(i);
    Eigen::Matrix<double,1,D> y,v,a;
    y.setZero();
    v.setZero();
    a.setZero();
    splineX.getEvalValue(i,y);
    splineX.getFirstDifference(i,v);
    splineX.getSecondDifference(i,a);
    file << i << "," << real.x() << "," << real.y() << "," << real.z() << ","
         << y(0,0) << "," << y(0,1) << "," << y(0,2) << ","
         << v(0,0) << "," << v(0,1) << "," << v(0,2) << ","
         << a(0,0) << "," << a(0,1) << "," << a(0,2) << std::endl;
  }

}