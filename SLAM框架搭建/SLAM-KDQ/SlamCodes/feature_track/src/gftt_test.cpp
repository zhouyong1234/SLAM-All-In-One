#include "zz_vio_interface/BottomImage.hpp"
#include "zz_vio_interface/IMU.hpp"
#include "optiflow_track/vins_optitrack.hpp"

int main(int argc,char** argv) {
  if(argc < 2) {
    std::cout << "./test file_path skip_time" << std::endl;
    return -1;
  }
  std::string replay_folder = argv[1];
  int count = atoi(argv[2]);
  OptiflowTrackByVINS vinsFeatTracker(25,0.1,30,replay_folder,1);
  cv::namedWindow("track",cv::WINDOW_NORMAL);
  for (size_t i = 1; i <= count; i++){
    std::string imageName = replay_folder + std::to_string(i) + ".jpg";
    cv::Mat image = cv::imread(imageName,CV_LOAD_IMAGE_GRAYSCALE);
   // cv::imshow("img",image);
    vinsFeatTracker.optiTrackFeature(0.1,image);
    std::cout << "Name " << imageName << std::endl;
   // cv::waitKey(0);  
  }
  
  return 0;
}
