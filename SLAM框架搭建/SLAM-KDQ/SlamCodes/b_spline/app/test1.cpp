#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <gsl/gsl_bspline.h>
using namespace std;

int main()
{
  std::string rawDataFile = "raw.csv";
  std::string newDataFile = "new.csv";
  std::ofstream rawFile(rawDataFile);
  std::ofstream newFile(newDataFile);
  rawFile << "t,raw" << std::endl;
  newFile << "t,new" << std::endl;
  size_t order = 4;
  size_t nbreaks = 10;
  // number of data points
  size_t ncoeffs = nbreaks + order - 2;
  size_t n = 20;
  gsl_bspline_workspace *bw;
  gsl_vector *B;
  std::vector<double> y;
  size_t i;
  B = gsl_vector_alloc(ncoeffs);
  // alloc a order-k bspline workspace
  bw = gsl_bspline_alloc(order, nbreaks);
  // use uniform breakpoints
  gsl_bspline_knots_uniform(0.0, 1.0, bw);

  for (i = 0; i < nbreaks; i++)
  {
    double xi = double(i) / nbreaks;
    double yi = sin(10.0 * xi);
    y.push_back(yi);
    rawFile << xi << "," << yi << std::endl;
  }
  for (i = 0; i < 4 * n; i++) {
    double xi = double(i)/ (4.0 * n);
    gsl_bspline_eval(xi,B,bw);
    size_t j;
    double yi = 0;
    for (j = 0; j < ncoeffs; j++)
    {
      double bi = gsl_vector_get(B,j);
      yi += bi * y[j];
    }
    newFile << xi << "," << yi << std::endl;
  }
  gsl_vector_free(B);
  gsl_bspline_free(bw);
  return 0;
}