//
// Created by kdq on 2021/6/16.
//
#pragma once

#include <datatable.h>
#include <bspline.h>
#include <bsplinebuilder.h>
#include <fstream>
#include <vector>
using namespace SPLINTER;
template <int S,int D,int K>
class BSplineX {
 public:
  static const int S_ = S;
  static const int D_ = D;
  static const int K_ = K;
  static_assert(K_ < 6);
  static_assert(S_ > 4);
  BSplineX(Eigen::Matrix<double,S_,1>& xVec,Eigen::Matrix<double,S_,D_>& yMatrix,double smoothAlpha = -1.0) {
    for (int i = 0; i < D_; i++) {
      DataTable samples;
      for (int j = 0; j < S_; j++) {
        samples.addSample(xVec(j,0),yMatrix(j,i));
      }
      if (smoothAlpha > 0.) {
        BSpline sp = BSpline::Builder(samples)
                              .smoothing(BSpline::Smoothing::PSPLINE)
                              .alpha(smoothAlpha)
                              .knotSpacing(BSpline::KnotSpacing::EXPERIMENTAL)
                              .degree(K_).build();
        splineVec.push_back(sp);
      } else {
        BSpline sp = BSpline::Builder(samples).degree(K_).build();
        splineVec.push_back(sp);
      }
    }
    xBegin_ = xVec(1,0);
    xEnd_ = xVec(S_ - 1,0);
  }

  bool getEvalValue(double x,Eigen::Matrix<double,1,D_>& y) {
    if ( x < xBegin_ || x > xEnd_) {
      return false;
    }
    DenseVector t(1);
    t(0) = x;
    for (int i = 0; i < D_; i++) {
      y(0,i) = splineVec[i].eval(t);
    }
    return true;
  }

  bool getFirstDifference(double x,Eigen::Matrix<double,1,D_>& v) {
    if ( x < xBegin_ || x > xEnd_) {
      return false;
    }
    DenseVector t(1);
    t(0) = x;
    for (int i = 0; i < D_; i++) {
      DenseMatrix vv;
      vv = splineVec[i].centralDifference(t);
      v(0,i) = vv(0,0);
    }
    return true;
  }

  bool getSecondDifference(double x,Eigen::Matrix<double,1,D_>& a) {
    if ( x < xBegin_ || x > xEnd_) {
      return false;
    }
    DenseVector t(1);
    t(0) = x;
    for (int i = 0; i < D_; i++) {
      DenseMatrix aa;
      aa = splineVec[i].secondOrderCentralDifference(t);
      a(0,i) = aa(0,0);
    }
    return true;
  }

 private:
  std::vector<BSpline> splineVec;
  double xBegin_,xEnd_;
};



