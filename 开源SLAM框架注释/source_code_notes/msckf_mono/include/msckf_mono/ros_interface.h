#ifndef MSCKF_MONO_ROS_INTERFACE_H_
#define MSCKF_MONO_ROS_INTERFACE_H_

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <msckf_mono/types.h>
#include <msckf_mono/msckf.h>
#include <msckf_mono/corner_detector.h>
#include <atomic>

namespace msckf_mono
{
class RosInterface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RosInterface(ros::NodeHandle nh);

  void imuCallback(const sensor_msgs::ImuConstPtr &imu);

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  void publish_core(const ros::Time &publish_time);

  void publish_extra(const ros::Time &publish_time);

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber image_sub_;
  image_transport::Publisher track_image_pub_;
  ros::Publisher odom_pub_;

  ros::Subscriber imu_sub_;

  void load_parameters();

  bool debug_;

  std::vector<std::tuple<double, imuReading<float>>> imu_queue_;
  double prev_imu_time_;

  void setup_track_handler();
  std::shared_ptr<corner_detector::TrackHandler> track_handler_;

  Matrix3<float> R_imu_cam_;
  Vector3<float> p_imu_cam_;

  Matrix3<float> R_cam_imu_; //相机与imu之间的旋转外参
  Vector3<float> p_cam_imu_; //相机与imu之间的移动外参

  std::string camera_model_;
  cv::Mat K_;                    //内参矩阵(3x3)
  std::string distortion_model_; //畸变模型
  cv::Mat dist_coeffs_;          //畸变系数

  int n_grid_cols_;
  int n_grid_rows_;
  float ransac_threshold_;

  enum CalibrationMethod
  {
    TimedStandStill
  };
  CalibrationMethod imu_calibration_method_;

  double stand_still_time_;
  double done_stand_still_time_;

  std::atomic<bool> imu_calibrated_;
  bool can_initialize_imu();
  void initialize_imu();

  int state_k_;
  void setup_msckf();
  MSCKF<float> msckf_;
  Camera<float> camera_;            //相机对象，记录相机内参和外参(相对于imu坐标系)
  noiseParams<float> noise_params_; //包含imu方差Q_imu, 状态初始协方差P_II_0|0
  MSCKFParams<float> msckf_params_;
  imuState<float> init_imu_state_;
};
} // namespace msckf_mono

#endif
