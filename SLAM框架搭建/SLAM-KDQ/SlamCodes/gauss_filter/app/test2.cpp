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
int main(void)
{
  const size_t N = 500;                        /* length of time series */
  const size_t K = 51;                         /* window size */
  const double alpha[3] = { 0.5, 3.0, 10.0 };  /* alpha values */
  GslGaussFilter f1(K,alpha[0]);
  GslGaussFilter f2(K,alpha[1]);
  GslGaussFilter f3(K,alpha[2]);
  std::vector<double> x;
  size_t i;
  double sum = 0.0;
  gsl_rng *r = gsl_rng_alloc(gsl_rng_default);
  /* generate input signal */
  for (i = 0; i < N; ++i)
  {
    double ui = gsl_ran_gaussian(r, 1.0);
    sum += ui;
    x.push_back(sum);
  }
  std::vector<double> y1,y2,y3;
  y1 = f1.apply(x);
  y2 = f2.apply(x);
  y3 = f3.apply(x);
  std::ofstream file("test2.csv",std::ios::out);
  file << "y,y0_5,y3,y10" << std::endl;
  /* print filter results */
  for (i = 0; i < N; ++i) {
    file << x[i] << "," << y1[i] << "," << y2[i] <<"," << y3[i] << std::endl;
  }

  return 0;
}
