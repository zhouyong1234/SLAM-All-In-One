#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <opencv/cv.hpp>
#include <opencv2/core/eigen.hpp>
class MotionParam {
  private:
  public:
    MotionParam(std::string configFile) {
      cv::FileStorage config(configFile,cv::FileStorage::READ);
      imuFreq_ = config["imuFreq"];
      imageFreq_ = config["imageFreq"];
      imuInterval_ = 1.0 / (double) imuFreq_;
      imageInterval_ = 1.0 / (double) imageFreq_;
      start_t_ = config["simStart"];
      end_t_ = config["simEnd"];
      gyr_noise_sigma_ = config["gyr_noise_sigma"];
      acc_noise_sigma_ = config["acc_noise_sigma"];
      gyr_bias_sigma_ = config["gyr_bias_sigma"];
      acc_bias_sigma_ = config["acc_bias_sigma"];
      pixel_noise_ = config["pixel_noise"];
      addPixelNoiseFlg_ = config["add_pixel_noise"];
      std::string cameraConfig = config["camera_yaml"];
      simLandmarkSize_ = config["simulator_landmark_size"];
      simLandmarkWidth_ = config["simulator_landmark_width"];
      simPositionNoiseXY_ = config["simulator_position_noise_xy"];
      simPositionNoiseZ_ = config["simulator_position_noise_z"];
      cameraConfigFile_ = cameraConfig;

      cv::Mat Tbc,tbc;
      Eigen::Matrix4d Tbc_tmp;
      config["Tbc"] >> Tbc;
      cv::cv2eigen(Tbc,Tbc_tmp);
      Rbc_ = Tbc_tmp.block<3,3>(0,0);
      tbc_ = Tbc_tmp.block<3,1>(0,3);

      cv::Mat ModelPara;
      config["SimModel"] >> ModelPara;
      Eigen::Matrix<double,3,6> SimModel;
      std::cout << "cv2eigen begin!" << std::endl;
      cv::cv2eigen(ModelPara,SimModel);
      std::cout << "cv2eigen ok!" << std::endl;
      for (size_t i = 0; i < 2; i++)
      {
        Skew_[i] = SimModel.col(3*i);
        Phase_[i] = SimModel.col(3*i + 1);
        Bias_[i] = SimModel.col(3*i + 2);
      }

      std::cout << "input ok!" << std::endl;
      std::cout << "imuFreq = " << imuFreq_ << std::endl;
      std::cout << "imageFreq = " << imageFreq_ << std::endl;
      std::cout << "Rbc = \n" << Rbc_ << std::endl;
      std::cout << "tbc = " << tbc_.transpose() << std::endl; 
      std::cout << "Skew = " << Skew_[1].transpose() << std::endl;
      std::cout << "Phase = " << Phase_[1].transpose() << std::endl;
      std::cout << "Bias = " << Bias_[1].transpose() << std::endl;
      std::cout << "SimLandmarkSize = " << simLandmarkSize_ << std::endl;
      std::cout << "SimLandmarkWidth = " << simLandmarkWidth_ << std::endl;
      std::cout << "simPositionNoiseXY = " << simPositionNoiseXY_ << std::endl;
      std::cout << "simPositionNoiseZ = " << simPositionNoiseZ_ << std::endl;
    };
    ~MotionParam(){};
    int imuFreq_,imageFreq_;
    double start_t_,end_t_;
    double gyr_noise_sigma_,acc_noise_sigma_;
    double gyr_bias_sigma_,acc_bias_sigma_;
    double pixel_noise_;
    double imuInterval_;
    double imageInterval_;
    Eigen::Matrix3d Rbc_;
    Eigen::Vector3d tbc_;
    Eigen::Vector3d Skew_[2];
    Eigen::Vector3d Phase_[2];
    Eigen::Vector3d Bias_[2];
    int addPixelNoiseFlg_;
    std::string cameraConfigFile_;
    int simLandmarkSize_ ;
    float simLandmarkWidth_;
    float simPositionNoiseXY_;
    float simPositionNoiseZ_;
};