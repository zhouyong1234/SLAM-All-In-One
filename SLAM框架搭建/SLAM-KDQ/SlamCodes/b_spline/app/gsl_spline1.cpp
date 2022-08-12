//
// Created by kdq on 2021/6/22.
//
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <Eigen/Core>
#include <gsl/gsl_bspline.h>
#include <fstream>
using namespace  std;

Eigen::Vector3d sampleY(double t,double lastT,Eigen::Vector3d lastData) {
  Eigen::Vector3d nowData;
  double dt = t - lastT;
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
  return nowData;
}

int main() {
  const int controlPointSize = 21;
  const int splineDegree = 6;
  const int nbreak = controlPointSize - splineDegree + 2;
  gsl_bspline_workspace *w = gsl_bspline_alloc(splineDegree,nbreak);
  gsl_vector *B;
  gsl_matrix *dB;
  B = gsl_vector_alloc(controlPointSize);
  dB = gsl_matrix_alloc(controlPointSize,splineDegree);
  double t0  = 0.;
  double t1 = 20.;
  gsl_bspline_knots_uniform(t0,t1,w);
  for (int i = 0;i < controlPointSize + splineDegree; i++) {
    std::cout << gsl_vector_get(w->knots,i) << "," ;
  }
  std::cout << std::endl;
  //get control points
  std::vector<Eigen::Vector3d> controlPoints,samplePoints;
  Eigen::Vector3d lastData = Eigen::Vector3d(2.0,1.0,2.0);
  int controlCnt = 0;
  double interval = 0.1;
  double lastT = t0;
  for (double t = t0; t < t1 + interval; t = t + interval) {
    Eigen::Vector3d nowData;
    nowData = sampleY(t,lastT,lastData);
    lastT = t;
    lastData = nowData;
    samplePoints.push_back(nowData);
    if (fabs(t - controlCnt) < 0.01 && controlCnt < controlPointSize) {
      controlPoints.push_back(nowData);
      controlCnt++;
      std::cout << t << ",";
    }
  }
  std::ofstream file("gslSpline.csv",std::ios::out);
  file << "t,p,v,a,bp,bv,ba\n";
  int i = 0;
  for (double t = t0; t < t1 + interval; t = t + interval) {

    gsl_bspline_eval(t,B,w);
    gsl_bspline_deriv_eval(t,1,dB,w);
    gsl_bspline_deriv_eval(t,2,dB,w);
    double p = 0.,v = 0.,a = 0.;
    for (int i = 0; i < controlPointSize; i++) {
      p += gsl_vector_get(B,i) * controlPoints[i].x();
      v += gsl_matrix_get(dB,i,1) * controlPoints[i].x();
      a += gsl_matrix_get(dB,i,2) * controlPoints[i].x();
    }
    Eigen::Vector3d nowData(p,v,a);
    file << t << "," << samplePoints[i].x() << "," << samplePoints[i].y() << ","<< samplePoints[i].z() << ","
         << nowData.x() << "," << nowData.y() << ","<< nowData.z() << std::endl;
    i++;
  }
  file.close();
  gsl_bspline_free(w);
  return 0;
}
