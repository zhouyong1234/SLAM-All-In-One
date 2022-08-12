#include <msckf_mono/ros_interface.h>

namespace msckf_mono
{
RosInterface::RosInterface(ros::NodeHandle nh) : nh_(nh),
                                                 it_(nh_),
                                                 imu_calibrated_(false),
                                                 prev_imu_time_(0.0)
{
  load_parameters();
  setup_track_handler();

  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 100);
  track_image_pub_ = it_.advertise("track_overlay_image", 1);

  imu_sub_ = nh_.subscribe("imu", 200, &RosInterface::imuCallback, this);
  image_sub_ = it_.subscribe("image_mono", 20,
                             &RosInterface::imageCallback, this);
}

void RosInterface::imuCallback(const sensor_msgs::ImuConstPtr &imu)
{
  double cur_imu_time = imu->header.stamp.toSec();
  if (prev_imu_time_ == 0.0)
  {
    prev_imu_time_ = cur_imu_time;
    done_stand_still_time_ = cur_imu_time + stand_still_time_;
    return;
  }

  imuReading<float> current_imu;

  current_imu.a[0] = imu->linear_acceleration.x;
  current_imu.a[1] = imu->linear_acceleration.y;
  current_imu.a[2] = imu->linear_acceleration.z;

  current_imu.omega[0] = imu->angular_velocity.x;
  current_imu.omega[1] = imu->angular_velocity.y;
  current_imu.omega[2] = imu->angular_velocity.z;

  current_imu.dT = cur_imu_time - prev_imu_time_;

  imu_queue_.emplace_back(cur_imu_time, current_imu);

  prev_imu_time_ = cur_imu_time;
}

void RosInterface::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  double cur_image_time = msg->header.stamp.toSec();
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // 转灰度图像
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (!imu_calibrated_)
  {
    if (imu_queue_.size() % 100 == 0)
    {
      ROS_INFO_STREAM("Has " << imu_queue_.size() << " readings");
    }

    if (can_initialize_imu())
    {
      //imu初始化，根据初始时刻imu测量信息均值设置imu初始状态,偏置，重力加速度等信息
      initialize_imu();

      imu_calibrated_ = true;
      imu_queue_.clear(); //标定数据不要

      //初始化msckf对象，设置相机模型，msckf配置参数，初始imu状态，初始imu噪声方差和初始状态协方差
      setup_msckf();
    }

    return;
  }

  std::vector<imuReading<float>> imu_since_prev_img;
  imu_since_prev_img.reserve(10);

  //获取上个图像帧到当前图像帧之间的imu信息，并移除掉过时imu帧
  // get the first imu reading that belongs to the next image
  auto frame_end = std::find_if(imu_queue_.begin(), imu_queue_.end(),
                                [&](const auto &x) { return std::get<0>(x) > cur_image_time; });

  std::transform(imu_queue_.begin(), frame_end,
                 std::back_inserter(imu_since_prev_img),
                 [](auto &x) { return std::get<1>(x); });

  imu_queue_.erase(imu_queue_.begin(), frame_end);

  for (auto &reading : imu_since_prev_img)
  {
    // 传播imu状态，并更新协方差矩阵，对应论文中III-B部分
    msckf_.propagate(reading);

    // 更新图像跟踪器中的陀螺仪角速度值，用于跟踪过程中的姿态估计
    Vector3<float> gyro_measurement = R_imu_cam_ * (reading.omega - init_imu_state_.b_g);
    track_handler_->add_gyro_reading(gyro_measurement);
  }

  // 更新图像跟踪器中的图像
  track_handler_->set_current_image(cv_ptr->image, cur_image_time);

  // 图像跟踪器跟踪特征
  std::vector<Vector2<float>,
              Eigen::aligned_allocator<Vector2<float>>>
      cur_features;
  corner_detector::IdVector cur_ids;
  track_handler_->tracked_features(cur_features, cur_ids);

  // 图像跟踪器产生新特征(fast角点)
  std::vector<Vector2<float>,
              Eigen::aligned_allocator<Vector2<float>>>
      new_features;
  corner_detector::IdVector new_ids;
  track_handler_->new_features(new_features, new_ids);

  // 状态向量的增广, 对应论文中III-C部分
  msckf_.augmentState(state_k_, (float)cur_image_time);
  // 更新跟踪特征点信息，并确定需要移除的特征点
  msckf_.update(cur_features, cur_ids);
  // 添加新特征点
  msckf_.addFeatures(new_features, new_ids);
  // 边缘化: 测量更新，对应论文中III-D,III-E部分
  msckf_.marginalize();
  // msckf_.pruneRedundantStates();
  msckf_.pruneEmptyStates();

  publish_core(msg->header.stamp);
  publish_extra(msg->header.stamp);
}

void RosInterface::publish_core(const ros::Time &publish_time)
{
  auto imu_state = msckf_.getImuState();

  nav_msgs::Odometry odom;
  odom.header.stamp = publish_time;
  odom.header.frame_id = "map";
  odom.twist.twist.linear.x = imu_state.v_I_G[0];
  odom.twist.twist.linear.y = imu_state.v_I_G[1];
  odom.twist.twist.linear.z = imu_state.v_I_G[2];

  odom.pose.pose.position.x = imu_state.p_I_G[0];
  odom.pose.pose.position.y = imu_state.p_I_G[1];
  odom.pose.pose.position.z = imu_state.p_I_G[2];
  Quaternion<float> q_out = imu_state.q_IG.inverse();
  odom.pose.pose.orientation.w = q_out.w();
  odom.pose.pose.orientation.x = q_out.x();
  odom.pose.pose.orientation.y = q_out.y();
  odom.pose.pose.orientation.z = q_out.z();

  odom_pub_.publish(odom);
}

void RosInterface::publish_extra(const ros::Time &publish_time)
{
  if (track_image_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage out_img;
    out_img.header.frame_id = "cam0";
    out_img.header.stamp = publish_time;
    out_img.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    out_img.image = track_handler_->get_track_image();
    track_image_pub_.publish(out_img.toImageMsg());
  }
}

bool RosInterface::can_initialize_imu()
{
  if (imu_calibration_method_ == TimedStandStill)
  {
    return prev_imu_time_ > done_stand_still_time_;
  }

  return false;
}

void RosInterface::initialize_imu()
{
  Eigen::Vector3f accel_accum;
  Eigen::Vector3f gyro_accum;
  int num_readings = 0;

  accel_accum.setZero();
  gyro_accum.setZero();

  // 累加初始一段时间内imu的测量值
  for (const auto &entry : imu_queue_)
  {
    auto imu_time = std::get<0>(entry);
    auto imu_reading = std::get<1>(entry);

    accel_accum += imu_reading.a;
    gyro_accum += imu_reading.omega;
    num_readings++;
  }

  // 计算初始静止时间段内加速度和角速度的平均值
  Eigen::Vector3f accel_mean = accel_accum / num_readings;
  Eigen::Vector3f gyro_mean = gyro_accum / num_readings;

  // 将静止时刻陀螺仪角速度均值作为角速度偏置
  init_imu_state_.b_g = gyro_mean;
  init_imu_state_.g << 0.0, 0.0, -9.81; //默认重力加速度
  // 由静止时刻imu加速度均值向量与默认重力加速度向量之间的轴角转换作为初始时刻imu状态的姿态信息
  init_imu_state_.q_IG = Quaternion<float>::FromTwoVectors(
      -init_imu_state_.g, accel_mean);

  init_imu_state_.b_a = init_imu_state_.q_IG * init_imu_state_.g + accel_mean; //0

  init_imu_state_.p_I_G.setZero(); //imu初始状态位置信息为0
  init_imu_state_.v_I_G.setZero(); //imu初始状态速度信息为0
  const auto q = init_imu_state_.q_IG;

  ROS_INFO_STREAM("\nInitial IMU State"
                  << "\n--p_I_G " << init_imu_state_.p_I_G.transpose() << "\n--q_IG " << q.w() << "," << q.x() << "," << q.y() << "," << q.x() << "\n--v_I_G " << init_imu_state_.v_I_G.transpose() << "\n--b_a " << init_imu_state_.b_a.transpose() << "\n--b_g " << init_imu_state_.b_g.transpose() << "\n--g " << init_imu_state_.g.transpose());
}

// 初始化跟踪器
void RosInterface::setup_track_handler()
{
  track_handler_.reset(new corner_detector::TrackHandler(K_, dist_coeffs_, distortion_model_));
  track_handler_->set_grid_size(n_grid_rows_, n_grid_cols_);
  track_handler_->set_ransac_threshold(ransac_threshold_);
}

// 初始化msckf对象
void RosInterface::setup_msckf()
{
  state_k_ = 0;
  msckf_.initialize(camera_, noise_params_, msckf_params_, init_imu_state_); //初始化msckf对象需要相机模型，噪声参数，msckf内部参数，初始imu状态
}

void RosInterface::load_parameters()
{
  std::string kalibr_camera;
  nh_.getParam("kalibr_camera_name", kalibr_camera);

  nh_.getParam(kalibr_camera + "/camera_model", camera_model_);

  K_ = cv::Mat::eye(3, 3, CV_32F);
  std::vector<float> intrinsics(4);
  nh_.getParam(kalibr_camera + "/intrinsics", intrinsics);
  K_.at<float>(0, 0) = intrinsics[0];
  K_.at<float>(1, 1) = intrinsics[1];
  K_.at<float>(0, 2) = intrinsics[2];
  K_.at<float>(1, 2) = intrinsics[3];

  nh_.getParam(kalibr_camera + "/distortion_model", distortion_model_);

  std::vector<float> distortion_coeffs(4);
  nh_.getParam(kalibr_camera + "/distortion_coeffs", distortion_coeffs);
  dist_coeffs_ = cv::Mat::zeros(distortion_coeffs.size(), 1, CV_32F);
  dist_coeffs_.at<float>(0) = distortion_coeffs[0];
  dist_coeffs_.at<float>(1) = distortion_coeffs[1];
  dist_coeffs_.at<float>(2) = distortion_coeffs[2];
  dist_coeffs_.at<float>(3) = distortion_coeffs[3];

  // 获取imu与相机外参
  XmlRpc::XmlRpcValue ros_param_list;
  nh_.getParam(kalibr_camera + "/T_cam_imu", ros_param_list);
  ROS_ASSERT(ros_param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  Matrix4<float> T_cam_imu;
  for (int32_t i = 0; i < ros_param_list.size(); ++i)
  {
    ROS_ASSERT(ros_param_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t j = 0; j < ros_param_list[i].size(); ++j)
    {
      ROS_ASSERT(ros_param_list[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      T_cam_imu(i, j) = static_cast<double>(ros_param_list[i][j]);
    }
  }

  R_cam_imu_ = T_cam_imu.block<3, 3>(0, 0);
  p_cam_imu_ = T_cam_imu.block<3, 1>(0, 3);

  R_imu_cam_ = R_cam_imu_.transpose();
  p_imu_cam_ = R_imu_cam_ * (-1. * p_cam_imu_);

  //　设置相机对象
  // setup camera parameters
  camera_.f_u = intrinsics[0];
  camera_.f_v = intrinsics[1];
  camera_.c_u = intrinsics[2];
  camera_.c_v = intrinsics[3];

  camera_.q_CI = Quaternion<float>(R_cam_imu_).inverse(); // TODO please check it
  camera_.p_C_I = p_cam_imu_;

  // Feature tracking parameteres
  nh_.param<int>("n_grid_rows", n_grid_rows_, 8);
  nh_.param<int>("n_grid_cols", n_grid_cols_, 8);

  float ransac_threshold_;
  nh_.param<float>("ransac_threshold_", ransac_threshold_, 0.000002);

  // MSCKF Parameters
  // 获取特征协方差
  float feature_cov;
  nh_.param<float>("feature_covariance", feature_cov, 7);

  // imu方差Q_imu对角线元素
  Eigen::Matrix<float, 12, 1> Q_imu_vars;
  float w_var, dbg_var, a_var, dba_var;
  nh_.param<float>("imu_vars/w_var", w_var, 1e-5);          //角速度方差
  nh_.param<float>("imu_vars/dbg_var", dbg_var, 3.6733e-5); //陀螺仪偏置随机游走方差
  nh_.param<float>("imu_vars/a_var", a_var, 1e-3);          //加速度方差
  nh_.param<float>("imu_vars/dba_var", dba_var, 7e-4);      //加速度计偏置随机游走方差
  Q_imu_vars << w_var, w_var, w_var,
      dbg_var, dbg_var, dbg_var,
      a_var, a_var, a_var,
      dba_var, dba_var, dba_var;

  // 初始imu状态(q, bg, v, ba, p)的协方差P_II_0|0对角线元素
  Eigen::Matrix<float, 15, 1> IMUCovar_vars;
  float q_var_init, bg_var_init, v_var_init, ba_var_init, p_var_init;
  nh_.param<float>("imu_covars/q_var_init", q_var_init, 1e-5);
  nh_.param<float>("imu_covars/bg_var_init", bg_var_init, 1e-2);
  nh_.param<float>("imu_covars/v_var_init", v_var_init, 1e-2);
  nh_.param<float>("imu_covars/ba_var_init", ba_var_init, 1e-2);
  nh_.param<float>("imu_covars/p_var_init", p_var_init, 1e-12);
  IMUCovar_vars << q_var_init, q_var_init, q_var_init,
      bg_var_init, bg_var_init, bg_var_init,
      v_var_init, v_var_init, v_var_init,
      ba_var_init, ba_var_init, ba_var_init,
      p_var_init, p_var_init, p_var_init;

  // Setup noise parameters
  // 设置imu方差Q_imu, 初始状态协方差P_II_0|0, 特征方差
  noise_params_.initial_imu_covar = IMUCovar_vars.asDiagonal();
  noise_params_.Q_imu = Q_imu_vars.asDiagonal();
  noise_params_.u_var_prime = pow(feature_cov / camera_.f_u, 2);
  noise_params_.v_var_prime = pow(feature_cov / camera_.f_v, 2);

  nh_.param<float>("max_gn_cost_norm", msckf_params_.max_gn_cost_norm, 11);
  msckf_params_.max_gn_cost_norm = pow(msckf_params_.max_gn_cost_norm / camera_.f_u, 2);
  nh_.param<float>("translation_threshold", msckf_params_.translation_threshold, 0.05); //移动阈值(如果跟踪特征点内的状态集合之间的最大距离超过该阈值则被认为是有效跟踪点)
  nh_.param<float>("min_rcond", msckf_params_.min_rcond, 3e-12);
  nh_.param<float>("keyframe_transl_dist", msckf_params_.redundancy_angle_thresh, 0.005); //关键帧移动距离的最小值
  nh_.param<float>("keyframe_rot_dist", msckf_params_.redundancy_distance_thresh, 0.05);  //关键帧旋转角度的最小值
  nh_.param<int>("max_track_length", msckf_params_.max_track_length, 1000);               //特征点被跟踪的最大次数(超过该次数将会被移除，并进行三角化)
  nh_.param<int>("min_track_length", msckf_params_.min_track_length, 3);                  //特征点被跟踪的最小次数
  nh_.param<int>("max_cam_states", msckf_params_.max_cam_states, 20);                     //最大相机状态数量

  // Load calibration time
  int method;
  nh_.param<int>("imu_initialization_method", method, 0);
  if (method == 0)
  {
    imu_calibration_method_ = TimedStandStill;
  }
  nh_.param<double>("stand_still_time", stand_still_time_, 8.0);

  ROS_INFO_STREAM("Loaded " << kalibr_camera);
  ROS_INFO_STREAM("-Intrinsics " << intrinsics[0] << ", "
                                 << intrinsics[1] << ", "
                                 << intrinsics[2] << ", "
                                 << intrinsics[3]);
  ROS_INFO_STREAM("-Distortion " << distortion_coeffs[0] << ", "
                                 << distortion_coeffs[1] << ", "
                                 << distortion_coeffs[2] << ", "
                                 << distortion_coeffs[3]);
  const auto q_CI = camera_.q_CI;
  ROS_INFO_STREAM("-q_CI \n"
                  << q_CI.x() << "," << q_CI.y() << "," << q_CI.z() << "," << q_CI.w());
  ROS_INFO_STREAM("-p_C_I \n"
                  << camera_.p_C_I.transpose());
}

} // namespace msckf_mono
