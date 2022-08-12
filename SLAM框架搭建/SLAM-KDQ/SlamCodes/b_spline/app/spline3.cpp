//
// Created by kdq on 2021/6/18.
// 这个项目主要为了检测b-spline是否能准确预测二阶导数和一阶导数
//
#include <iostream>
#include "BSplineX.hpp"


Eigen::Vector3d lastData = Eigen::Vector3d(2.,4.,0);
double lastT = 2.;

Eigen::Vector3d sampleY(double t) {
  Eigen::Vector3d nowData;
  double dt = t - lastT;
  lastT = t;
  if ( t < 5 ) {
    nowData.z() = 2;
  } else if (t < 8) {
    nowData.z() = 0;
  } else if (t < 11) {
    nowData.z() = -2;
  } else if (t < 14) {
    nowData.z() = 0;
  } else if (t < 17) {
    nowData.z() = 2;
  } else {
    nowData.z() = 0;
  }

  double acc =  0.5 * (lastData.z() + nowData.z());
  nowData.x() = lastData.x() + lastData.y() * dt + 0.5 * acc * dt * dt;
  nowData.y() = lastData.y() + acc * dt;
  lastData = nowData;
  return nowData;
}

int main() {
  const int D = 1;
  const int S = 20;
  const int K = 5;
  Eigen::Matrix<double, S, 1> x;
  Eigen::Matrix<double, S, 1> y;
  std::ofstream file("spline3.csv", std::ios::out);
  int j = 0;
  for (double i = 2.0 + 1; i < 2.0 + S + 1; i++) {
    Eigen::Vector3d yy = sampleY(i);
    x(j, 0) = i;
    y(j, 0) = yy.x();
    j++;
    file << i << "," << yy.x() << "," << yy.y() << "," << yy.z() << std::endl;
  }
  file << "-----------------------------------------------------------------" << std::endl;

  BSplineX<S, D, K> splineX(x, y);
  for (double i = 4.; i < 2.0 + S - 1; i += 0.1) {
    Eigen::Matrix<double, 1, D> outy, outv, outa;
    splineX.getEvalValue(i, outy);
    splineX.getFirstDifference(i, outv);
    splineX.getSecondDifference(i, outa);
    file << i << "," << outy(0, 0) << "," << outv(0, 0) << "," << outa(0, 0) << std::endl;
  }
}
