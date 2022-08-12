//
// Created by kdq on 2021/6/23.
//
#include <list>
#include <iostream>
#include <fstream>
#include "BSplineX.hpp"

const int D = 3;
const int S = 15;
const int K = 5;
std::ofstream recordFile("spline4.csv",std::ios::out);
double lastTime = 0;
void splineAccel(const std::list<std::pair<double,Eigen::Vector3d> >& slideWindow) {

  if (slideWindow.size() != S) {
    std::cout << "Input data size != S" << std::endl;
    return;
  }

  Eigen::Matrix<double, S, 1> x;
  Eigen::Matrix<double, S, D> y;
  int cnt = 0;
  for (auto it = slideWindow.begin();it != slideWindow.end(); it++) {
    std::pair<double,Eigen::Vector3d> data = *it;
    x(cnt,0) = data.first;
    y.row(cnt) = data.second;
    cnt++;
  }
  BSplineX<S,D,K> spline(x,y);
  double t0 = x(2,0);
  double t1 = x(S-2,0);
  for (double t = t0; t < t1; t = t + 0.01) {
    Eigen::Matrix<double, 1, D> outy, outv, outa;
    if (lastTime > t) {
      continue;
    }
    lastTime = t;
    spline.getEvalValue(t, outy);
    spline.getFirstDifference(t, outv);
    spline.getSecondDifference(t, outa);
    recordFile << t << ","
               << outy(0, 0) << "," << outy(0, 1) << ","  << outy(0, 2) << ","
               << outv(0, 0) << "," << outv(0, 1) << ","  << outv(0, 2) << ","
               << outa(0, 0) << "," << outa(0, 1) << ","  << outa(0, 2) << std::endl;
  }
}

int main(int argc,char **argv) {
  if (argc !=2 ) {
    std::cout << "Please input data file!" << std::endl;
    return -1;
  }
  recordFile << "t,px,py,pz,vx,vy,vz,ax,ay,az\n";
  std::ifstream file;
  std::string filename = argv[1];
  file.open(filename,std::ios::in);
  if (!file.is_open()) {
    std::cout << "Open File failure!" << std::endl;
    return -1;
  }
  std::vector<double> x,y,z,t;
  while (!file.eof() && file.good()) {
    std::string lineStr;
    getline(file,lineStr);
    std::stringstream s;
    s.str(lineStr);
    double tt,xx,yy,zz;
    char s1,s2,s3;
    s >> tt >> s1 >> xx >> s2 >> yy >> s3 >> zz; //if line string is not number,"tt,xx,yy,zz" will be zero not nan
    if (std::isnan(tt) || std::isnan(xx) || std::isnan(yy) || std::isnan(zz)) {
      continue;
    }
    t.push_back(tt);
    x.push_back(xx);
    y.push_back(yy);
    z.push_back(zz);
  }
  std::list<std::pair<double,Eigen::Vector3d> > slideWindow;
  size_t div = 1; //[6-10]
  //skip i = 0 for first line in the sample.txt is "t,tx,ty,tz"
  for (int i = 1; i < t.size(); i++) {
    if (i % div != 0) {
      continue;
    }
    slideWindow.push_back(std::make_pair(t[i],Eigen::Vector3d(x[i],y[i],z[i])));
    if (slideWindow.size() == S) {
        splineAccel(slideWindow);
        slideWindow.clear();
    }
  }
}