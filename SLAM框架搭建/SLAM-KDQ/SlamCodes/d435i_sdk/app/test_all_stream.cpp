//
// Created by kdq on 2021/5/21.
//

#include "d435i.hpp"

int main(int argc,char **argv) {
  D435I* d435i;
  if (argc < 2) {
    std::cout << "you can input config file,otherwise use default config paramters!" << std::endl;
    d435i = new D435I();
  } else {
    d435i = new D435I(argv[1]);
  }
  d435i->start();
  while(1) {
    StereoStream stereoInfos;
    ImuStream imuInfos;
    if (d435i->getD435IStreamDatas(stereoInfos,imuInfos)) {
    };
    cv::waitKey(1);
  }
  return 0;
}