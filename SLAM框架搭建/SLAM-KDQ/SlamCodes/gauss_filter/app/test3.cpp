//
// Created by kdq on 2021/6/22.
//

//
// Created by kdq on 2021/6/22.
//
#include <stdio.h>
#include <stdlib.h>
#include "GslGaussFilter.hpp"
#include <gsl/gsl_math.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include "fstream"
#include "iostream"
int main(void)
{
  const size_t N = 500;                        /* length of time series */
  const size_t K = 51;                         /* window size */
  GslGaussFilter f1(K,1.0);

  std::vector<Eigen::Vector3d> x;
  size_t i;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  gsl_rng *r = gsl_rng_alloc(gsl_rng_default);
  /* generate input signal */
  for (i = 0; i < N; ++i)
  {
    double u1 = gsl_ran_gaussian(r, 1.0);
    double u2 = gsl_ran_gaussian(r, 0.5);
    double u3 = gsl_ran_gaussian(r, 2.0);
    sum += Eigen::Vector3d(u1,u2,u3);
    x.push_back(sum);
  }
  std::vector<Eigen::Vector3d> y;
  y = f1.apply(x);
  std::ofstream file("test3_temp.csv",std::ios::out);
  file << "x1,x2,x3,,y1,y2,y3" << std::endl;
  /* print filter results */
  for (i = 0; i < N; ++i) {
    file << x[i](0) << "," << x[i](1) << "," << x[i](2) << "," << y[i](0) << "," << y[i](1) <<"," << y[i](2) << std::endl;
  }

  return 0;
}
