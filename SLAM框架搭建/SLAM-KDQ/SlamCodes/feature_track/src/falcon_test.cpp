
#include "rovio.pb.h"
#include "image.pb.h"
#include "replaykit.h"
#include "optiflow_track/vins_optitrack.hpp"
#include <opencv2/opencv.hpp>
using namespace std;

void test_remap(cv::Mat& mapx,cv::Mat& mapy) {
  mapx.create(240,320,CV_32FC1);
  mapy.create(240,320,CV_32FC1);
  for (size_t x = 0; x < 320;x++) 
    for (size_t y = 0;y < 240;y++) {
      if (y > 40 && y < 200)
        mapy.at<float>(y,x) = y;
      else 
        mapy.at<float>(y,x) = 240 - y;
      mapx.at<float>(y,x) = x;  
    }
}

typedef ::zz::replaykit::ReplayKit<
    ::zz::replaykit::Topics<vision::BottomImage, rovio::InputInfoPack>,
    ::zz::replaykit::Commands<>>
    ReplayKitType;
int main(int argc,char** argv) {
  std::cout << "Hello replayKit optflow!" << std::endl;
  if(argc < 2) {
    cout << "./replayFalcon rovioimageFile savePath skipTime playSpeed" << std::endl;
  }
  string fileName = argv[1]; 
  string imageStorePath = argc < 3 ? "IMAGE" : argv[2];
  double replay_skip_time = argc < 4 ? 0 : atof(argv[3]);
  float replay_speed = argc < 5 ? 1 : atof(argv[4]);
  ReplayKitType replaykit;
  OptiflowTrackByVINS vinsFeatTracker(30,0.1,30,imageStorePath,1);
  // cv::Mat mapx,mapy;
  // test_remap(mapx,mapy);
  replaykit.Subscribe<0>([&](double now_time, const vision::BottomImage &bottomImage) {
    const cv::Mat im = cv::Mat(cv::Size(bottomImage.width(), bottomImage.height()), CV_8UC1,
                               (char *) bottomImage.image_buffer().c_str()).clone();
    auto delay = static_cast<float>(now_time - bottomImage.timestamp());
    auto exp_time = bottomImage.exposure_time();
    // cv::Mat img;
    // cv::remap(im,img,mapx,mapy,cv::INTER_LINEAR);
    if (1) {
      printf("[Image] Get %12.6f at %12.6f, exp = %7.3fms, delay = %7.3fms\n",
             bottomImage.timestamp(), now_time, exp_time * 1e3f, delay * 1e3f);
    }
  vinsFeatTracker.optiTrackFeature(bottomImage.timestamp() + exp_time * 0.5,im);
    //rovioNode.imgCallback(bottomImage.timestamp() + exp_time * 0.5, im, delay);
  });

  zz::replaykit::FileReplayReader<ReplayKitType> reader(fileName, replaykit, replay_speed);
  reader.SetStartTime(replay_skip_time);
  std::thread replay_thread([&]() {
    replaykit.Start();
  });
  reader.Start();
  printf("Rovio replay finished!\n");
  replay_thread.join();
}