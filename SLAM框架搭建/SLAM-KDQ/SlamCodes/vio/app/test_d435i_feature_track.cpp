#include "d435i.hpp"
#include "unistd.h"
#include "Camera.hpp"
#include "FeatureTracker.hpp"
using namespace vio;

int main(int argc,char **argv) {
  if (argc < 2) {
    std::cerr << "please input config file!" << std::endl;
    return -1;
  }
  std::string cfgFile = argv[1];
  Config* cfg = new Config(cfgFile);
  CameraPtr cam(new Camera(cfg));
  FeatureTracker *featTracker = new FeatureTracker(cfg);
  FramePtr lastF = nullptr,curF = nullptr;
  D435I d435i_device(cfgFile);
  d435i_device.start();
  std::list<std::map<uint64_t,cv::Point2f> > windowsFeatures;
  while (1) {
    double timestamp;
    cv::Mat leftImg,rightImg;
    if (d435i_device.getInfraredImages(timestamp,leftImg,rightImg)) {
      curF.reset(new Frame(timestamp,leftImg,cam));
      cv::Mat R_cur_last;
      featTracker->detectAndTrackFeature(lastF,curF,R_cur_last);
      lastF = curF;
    }
  }
}
