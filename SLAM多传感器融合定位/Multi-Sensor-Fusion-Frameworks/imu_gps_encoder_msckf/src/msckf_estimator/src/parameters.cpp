#include "parameters.h"

std::string IMU_TOPIC;
std::string GPS_TOPIC;
std::string ENCODER_TOPIC;
Eigen::Vector3d t_o_g;
Eigen::Matrix3d R_o_i;
Eigen::Vector3d t_o_i;
Eigen::Matrix3d R_o_c;
Eigen::Vector3d t_o_c;
double acc_noise;
double acc_bias_noise;
double gyro_noise;
double gyro_bias_noise;
double left_wheel_diameter;
double right_wheel_diameter;
double wheel_base;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name){
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n){
    std::string config_file = readParam<std::string>(n, "config_file");
	cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

	fsSettings["imu_topic"] >> IMU_TOPIC;
	fsSettings["gps_topic"] >> GPS_TOPIC;
	fsSettings["encoder_topic"] >> ENCODER_TOPIC;

	cv::Mat cv_t_o_g;
    fsSettings["Extrinsic.t_o_g"] >> cv_t_o_g;
	cv::cv2eigen(cv_t_o_g, t_o_g);
	cv::Mat cv_R_o_i;
    fsSettings["Extrinsic.R_o_i"] >> cv_R_o_i;
	cv::cv2eigen(cv_R_o_i, R_o_i);
	cv::Mat cv_t_o_i;
    fsSettings["Extrinsic.t_o_i"] >> cv_t_o_i;
	cv::cv2eigen(cv_t_o_i, t_o_i);
	cv::Mat cv_R_o_c;
    fsSettings["Extrinsic.R_o_c"] >> cv_R_o_c;
	cv::cv2eigen(cv_R_o_c, R_o_c);
	cv::Mat cv_t_o_c;
    fsSettings["Extrinsic.t_o_c"] >> cv_t_o_c;
	cv::cv2eigen(cv_t_o_c, t_o_c);

	fsSettings["acc_noise"] >> acc_noise;
	fsSettings["acc_bias_noise"] >> acc_bias_noise;
	fsSettings["gyro_noise"] >> gyro_noise;
	fsSettings["gyro_bias_noise"] >> gyro_bias_noise;

	fsSettings["left_wheel_diameter"] >> left_wheel_diameter;
	fsSettings["right_wheel_diameter"] >> right_wheel_diameter;
	fsSettings["wheel_base"] >> wheel_base;
}