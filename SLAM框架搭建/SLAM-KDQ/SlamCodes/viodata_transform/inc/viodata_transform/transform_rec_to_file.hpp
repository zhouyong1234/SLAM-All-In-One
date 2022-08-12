#include <iostream>
#include <sys/stat.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

struct RecImageType {
  RecImageType(int64 t,const cv::Mat& img){
    timestamp = t;
    image = img.clone();
  }
  int64_t timestamp;
  cv::Mat image;
};

struct RecImuType {
  RecImuType(int64_t t,Eigen::Vector3d a,Eigen::Vector3d g) {
    timestamp = t;
    acc = a;
    gyr = g;
  }
  int64_t timestamp;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyr;  
};




class TransformRec2File {
  private:
    std::string imageRecordPath_;
    std::string imuRecordPath_;
    std::fstream imageNameFile_;
    std::fstream imuDataFile_;
  public:
  TransformRec2File(std::string filePath) {
    imageRecordPath_ = filePath + "/rec2file/img/";
    imuRecordPath_ = filePath + "/rec2file/imu/";
    struct stat info{};
    if (stat(imageRecordPath_.c_str(), &info) == 0 ) {
      std::string cmd_rm("rm -rf " + filePath + "/rec2file");
      int status_rm = system(cmd_rm.c_str());
      assert(WIFEXITED(status_rm));
    }
    std::string cmd("mkdir -p " + imageRecordPath_);
    int status = system(cmd.c_str());
    assert(WIFEXITED(status));
    std::string cmd2("mkdir -p " + imuRecordPath_);
    int status2 = system(cmd2.c_str());
    assert(WIFEXITED(status2));
    imageNameFile_.open(imageRecordPath_ + "img.txt",std::ios_base::out);
    imuDataFile_.open(imuRecordPath_ + "imu.txt",std::ios_base::out);   
  };

  ~TransformRec2File() {
    imageNameFile_.close();
    imuDataFile_.close();
  };

  void transformImageData(const RecImageType& img) {
    std::stringstream t_str;
    t_str << img.timestamp;
    cv::imwrite(imageRecordPath_ + t_str.str() + ".png",img.image);
    imageNameFile_ << t_str.str() << std::endl;
  }

  void transformImuData(const RecImuType& imu)  {
    std::stringstream t_str;
    t_str << imu.timestamp << " " << imu.acc.x() << " " << imu.acc.y() << " " << imu.acc.z() << " " 
          << imu.gyr.x() << " " << imu.gyr.y() << " " << imu.gyr.z() << " " << std::endl;
    imuDataFile_ << t_str.str();
  }
};
