#pragma once
#include <iostream>
#include <string>
#include <map>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "imu_motion/imu_motion.hpp"

struct ProjectPointInfo {
  double t;
  std::map<int,std::pair<cv::Point3d,cv::Point2i>> ptsMap;
  Eigen::Isometry3d Twc;
};


class VioDatasInterface
{
private:
  
public:
  /** @brief Constructor 
   * 
   */ 
  VioDatasInterface(/* args */);

  /** @brief Destructor
   * 
   */ 
  ~VioDatasInterface();
 
  /** @brief record imu state include pose vel attitude acc gyro etc.
   *  @param data - imu state
   *  @param fileName - recording file name
   *  @param title - true: record title; false : record data
   */ 
  static void recordImuMotionState(const ImuMotionState& data,const std::string fileName,bool title = false) {
    if(title) {
      std::fstream file(fileName,std::ios::out);
      file << "t px py pz qw qx qy qz vx vy vz ax ay az wx wy wz bax bay baz bwx bwy bwz\n";
      return;
    }
    std::fstream file(fileName,std::ios::app);
    file << data.timestamp_ << " "
         << data.pos_.x() << " " 
         << data.pos_.y() << " " 
         << data.pos_.z() << " " 
         << data.qwi_.w() << " "
         << data.qwi_.x() << " "
         << data.qwi_.y() << " "
         << data.qwi_.z() << " "
         << data.vel_.x() << " "
         << data.vel_.y() << " "
         << data.vel_.z() << " "
         << data.acc_.x() << " "
         << data.acc_.y() << " "
         << data.acc_.z() << " "
         << data.gyr_.x() << " "
         << data.gyr_.y() << " "
         << data.gyr_.z() << " "
         << data.acc_bias_.x() << " "
         << data.acc_bias_.y() << " "
         << data.acc_bias_.z() << " "
         << data.gyr_bias_.x() << " "
         << data.gyr_bias_.y() << " "
         << data.gyr_bias_.z() << "\n";
  }

  /** @brief read imu state from file
   *  @param dateVec - vector of ImuMotionState
   *  @param fileName - file recorded with imu state
   */  
  static void readImuMotionState(std::vector<ImuMotionState>& dataVec,const std::string fileName) {
    std::fstream file(fileName,std::ios::in);
    if (file.is_open()) {
      std::string tmp;
      std::getline(file,tmp);
      while(!file.eof()) {
        ImuMotionState data;
        file >> data.timestamp_;
        file >> data.pos_[0]; 
        file >> data.pos_[1]; 
        file >> data.pos_[2];
        file >> data.qwi_.w();
        file >> data.qwi_.x();
        file >> data.qwi_.y();
        file >> data.qwi_.z();
        file >> data.vel_[0]; 
        file >> data.vel_[1]; 
        file >> data.vel_[2];  
        file >> data.acc_[0]; 
        file >> data.acc_[1]; 
        file >> data.acc_[2];  
        file >> data.gyr_[0]; 
        file >> data.gyr_[1]; 
        file >> data.gyr_[2];  
        file >> data.acc_bias_[0]; 
        file >> data.acc_bias_[1]; 
        file >> data.acc_bias_[2];  
        file >> data.gyr_bias_[0]; 
        file >> data.gyr_bias_[1]; 
        file >> data.gyr_bias_[2];
        dataVec.push_back(data);
      }
      dataVec.erase(dataVec.end()-1);
    } else {
      std::cout << fileName << " can't be open" << std::endl;
    }
  }

  /** @brief record pose and attitude of imu with tum format for evo evaluate
    * @param data - imu state
    * @param fileName - file used to record pose
    * @param title - 1: record tilte ; 0 - record data
   */ 
  static void recordPoseAsTum(const ImuMotionState& data,const std::string fileName,bool title = false) {
    if(title) {
      std::fstream file(fileName,std::ios::out);
      file << "t px py pz qx qy qz qw\n";
      return;
    }
    std::fstream file(fileName,std::ios::app);
        file.precision(9);
        file << data.timestamp_ << " ";
        file.precision(5);
        file << data.pos_.x() << " "
             << data.pos_.y() << " "
             << data.pos_.z() << " "
             << data.qwi_.x() << " "
             << data.qwi_.y() << " "
             << data.qwi_.z() << " "
             << data.qwi_.w() <<std::endl;
  }

  /** @brief record 3D point and pixel infos 
    * @param ptsInfo - which include camera pose,timestamp,3D points' coordinates in world and corrsponding pixel coordinates
    * @param fileName - file used to record infos
    * @param title - 1: record tilte ; 0 - record data
   */   
  static void recordCameraPixel(const ProjectPointInfo& ptsInfo,const std::string fileName,bool title = false) {
    if (title) {
      std::fstream file(fileName,std::ios::out);
      file << "t,id p1W_x p1W_y p1W_z pix1_x pix1_y, ...\n"; 
      return;
    }
    std::fstream file(fileName,std::ios::app);
    file.precision(9);
    Eigen::Quaterniond q(ptsInfo.Twc.rotation());
    Eigen::Vector3d p(ptsInfo.Twc.translation());
    file << ptsInfo.t << "," << p.x() << " " << p.y() << " " << p.z() << " " 
         << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "," ;
    std::map<int,std::pair<cv::Point3d,cv::Point2i> >::const_iterator it;
    for (it = ptsInfo.ptsMap.begin();it != ptsInfo.ptsMap.end();it++) {
      std::pair<cv::Point3d,cv::Point2i> pt = it->second;
      file << it->first << " " << pt.first.x << " " << pt.first.y << " " << pt.first.z << " " << pt.second.x << " " << pt.second.y << ","; 
    }
    file << "\n";
  }

  /** @brief read points infos
    * @param fileName - file which record all infos
    * @param proInfoVec - vector used to store all infos 
   */ 
  static void readCameraPixel(std::string fileName,std::vector<ProjectPointInfo>& proInfoVec) {
    std::fstream file(fileName,std::ios::in);
    if(!file) {
      std::cout << "can't openc file " << fileName << std::endl;
    }
    std::string strLine;
    std::getline(file,strLine);
    while (!file.eof())
    {
      ProjectPointInfo ptsInfo;
      std::getline(file,strLine);
      if (strLine.length() == 0) {
        continue;
      }
      std::stringstream ss(strLine);
      std::string tokenStr;
      std::getline(ss,tokenStr,',');
      std::stringstream tt(tokenStr);
      tt >> ptsInfo.t;
      std::getline(ss,tokenStr,',');
      std::stringstream pp(tokenStr);
      Eigen::Quaterniond quat;
      Eigen::Vector3d trans;
      pp >> trans(0);
      pp >> trans(1);
      pp >> trans(2);
      pp >> quat.w();
      pp >> quat.x();
      pp >> quat.y();
      pp >> quat.z();
      ptsInfo.Twc.setIdentity();
      ptsInfo.Twc.prerotate(quat);
      ptsInfo.Twc.pretranslate(trans);   
      while(std::getline(ss,tokenStr,',')) {
        std::stringstream subStr(tokenStr);
        int id;
        cv::Point3f p3W;
        cv::Point2i uv;
        subStr >> id;
        subStr >> p3W.x;
        subStr >> p3W.y;
        subStr >> p3W.z;
        subStr >> uv.x;
        subStr >> uv.y;
        ptsInfo.ptsMap[id] = std::make_pair(p3W,uv);
      }
      proInfoVec.push_back(ptsInfo);
    }
  }
};
