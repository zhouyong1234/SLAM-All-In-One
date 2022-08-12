//
// Created by kdq on 2021/11/8.
//

#include "DepthFilter.hpp"
using namespace depth_filter;
struct DepthInfo {
  DepthInfo(float dd,float dder) {
    d = dd;
    dErr = dder;
  }
  float d;
  float dErr;
};
int main() {
  Seed seed(30.,10.,2,10);
  DepthFilter DFilter;

  std::vector<DepthInfo> meas;
  meas.emplace_back(100,10);
  meas.emplace_back(120,20);
  meas.emplace_back(98,30);
  meas.emplace_back(86,25);
  meas.emplace_back(25,10);
  meas.emplace_back(30,15);
  meas.emplace_back(98,43);
  meas.emplace_back(120,30);
  meas.emplace_back(87,17);
  meas.emplace_back(132,50);
  for(size_t i = 0; i < 10; i++) {
    float x = meas[i].d;
    float xerr = meas[i].dErr;
    float tau = 0.5 * (1./fmax(1e-7,x - xerr) - 1./(x + xerr));
    DFilter.updateSeed(1./x,tau*tau,&seed);
  }
}