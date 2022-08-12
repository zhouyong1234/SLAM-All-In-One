#include "zz_vio_interface/BottomImage.hpp"
#include "zz_vio_interface/IMU.hpp"
#include "optiflow_track/vins_optitrack.hpp"

int main(int argc,char** argv) {
  if(argc < 2) {
    std::cout << "./test file_path skip_time" << std::endl;
    return -1;
  }


  std::string replay_folder = argv[1];
  OptiflowTrackByVINS vinsFeatTracker(25,0.03,40,replay_folder,1);
  double replay_skip_time = 85;
  if(argc == 3) {
    std::string skip_time = argv[2];
     std::stringstream ss(skip_time);
     ss >> replay_skip_time;
  }
  nnstation::BottomClient bottomClient;
  bottomClient.setImageSize(320, 240);
  bottomClient.subscribe([&](const nnstation::BottomClient::mtParsed &parsed) {
    if(!parsed.img.empty()) {
      cv::imshow("bottom",parsed.img);
      cv::waitKey(1);
    }
    vinsFeatTracker.optiTrackFeature(parsed.t,parsed.img);
    printf("[Image] Get %12.6fms\n", parsed.t);
  });
  std::cout << "hello interface!" << std::endl;

  nnstation::ImuClient imuClient;
  imuClient.subscribe([&](const nnstation::ImuClient::mtParsed &parsed) {
    for (const auto &info : parsed) {
//      printf("[IMU]%12.6f,%f,%f,%f,%f,%f,%f\n",
//             info.t, info.gyr.x(), info.gyr.y(), info.gyr.z(), info.acc.x(), info.acc.y(), info.acc.z());
      printf("[IMU]:%12.6f,%f\n",info.t,info.proxi);
      usleep(1);
    }
  });
  if (!replay_folder.empty()) {
    std::cout << replay_folder << std::endl;
    auto t1 = DBL_MAX, t2 = DBL_MAX;//, t3 = DBL_MAX;
    bottomClient.getOldestReplayTime(replay_folder + "/img.rec", t1);
    imuClient.getOldestReplayTime(replay_folder + "/imu.rec", t2);
    double max_data_t = std::max(t1, t2) - 2.0;
    double start_data_t = std::max(max_data_t, replay_skip_time);
    printf("Oldest bottom time: %20.9f\n", t1);
    printf("Oldest imu time: %20.9f\n", t2);
    std::cout << "start replay since: "<< start_data_t << std::endl;
    auto start_real_t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    bottomClient.startReplay(replay_folder + "/img.rec", start_real_t, start_data_t);
    imuClient.startReplay(replay_folder + "/imu.rec", start_real_t, start_data_t);
  }
  sleep(UINT32_MAX); 
  return 0;
}
