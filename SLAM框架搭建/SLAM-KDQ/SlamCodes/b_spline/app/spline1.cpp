//
// Created by kdq on 2021/6/16.
//

#include <iostream>
#include <datatable.h>
#include <bspline.h>
#include <bsplinebuilder.h>
#include <fstream>

using std::cout;
using std::endl;

using namespace SPLINTER;

// Six-hump camelback function
double f(double x)
{
  return 2 * sin(x);
}

int main(int argc, char *argv[])
{
  std::ofstream file("data.csv",std::ios::out);
  // Create new DataTable to manage samples
  DataTable samples;

  // Sample the function
  DenseVector x(1);
  for(int i = 0; i < 20; i++)
  {
    x(0) = i;
    samples.addSample(x,f(x(0)));
  }

  // Build B-splines that interpolate the samples
  BSpline bspline1 = BSpline::Builder(samples).degree(1).build();
  BSpline bspline3 = BSpline::Builder(samples).degree(5).build();

  // Build penalized B-spline (P-spline) that smooths the samples
  BSpline pspline = BSpline::Builder(samples)
    .degree(3)
    .smoothing(BSpline::Smoothing::PSPLINE)
    .alpha(0.03)
    .build();

  DenseMatrix v,a;
  for (int i = 0; i < 200; i++) {
    x(0) = i * 0.1;
    double d = f(x(0));
    double y = bspline3.eval(x);
    v = bspline3.centralDifference(x);
    a = bspline3.secondOrderCentralDifference(x);
    file << x(0) << "," << d << "," << y << "," << v(0,0) << "," << a(0,0) << std::endl;
  }

  return 0;
}