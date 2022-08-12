#include <QMutexLocker>

#include "ROSThread.h"

using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), mutex_(th_mutex)
{

  processed_stamp_ = 0;
  play_rate_ = 1.0;
  loop_flag_ = false;
  stop_skip_flag_ = true;
  stereo_active_ = true;
  omni_active_ = false;
  search_bound_ = 10;
  reset_process_stamp_flag_ = false;
  auto_start_flag_ = true;
  stamp_show_count_ = 0;
  imu_data_version_ = 0;
  encoder_resolution_ = 0;
  encoder_left_diameter_ = 0.0;
  encoder_right_diameter_ = 0.0;
  encoder_wheel_base_ = 0.0;
  encoder_param_load_flag_ = false;
  encoder_x_ = 0.0;
  encoder_y_ = 0.0;
  encoder_theta_ = 0.0;
  prev_clock_stamp_ = 0;

}

ROSThread::~ROSThread()
{

  data_stamp_thread_.active_ = false;
  altimter_thread_.active_ = false;
  encoder_thread_.active_ = false;
  fog_thread_.active_ = false;
  gps_thread_.active_ = false;
  vrs_thread_.active_ = false;
  imu_thread_.active_ = false;
  velodyne_left_thread_.active_ = false;
  velodyne_right_thread_.active_ = false;

  usleep(100000);

  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();

  altimter_thread_.cv_.notify_all();
  if(altimter_thread_.thread_.joinable()) altimter_thread_.thread_.join();

  encoder_thread_.cv_.notify_all();
  if(encoder_thread_.thread_.joinable()) encoder_thread_.thread_.join();

  fog_thread_.cv_.notify_all();
  if(fog_thread_.thread_.joinable()) fog_thread_.thread_.join();

  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();

  vrs_thread_.cv_.notify_all();
  if(vrs_thread_.thread_.joinable()) vrs_thread_.thread_.join();

  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();

  velodyne_left_thread_.cv_.notify_all();
  if(velodyne_left_thread_.thread_.joinable()) velodyne_left_thread_.thread_.join();

  velodyne_right_thread_.cv_.notify_all();
  if(velodyne_right_thread_.thread_.joinable()) velodyne_right_thread_.thread_.join();

}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
  nh_ = n;

  pre_timer_stamp_ = ros::Time::now().toNSec();
  timer_ = nh_.createTimer(ros::Duration(0.0001), boost::bind(&ROSThread::TimerCallback, this, _1));

  start_sub_  = nh_.subscribe<std_msgs::Bool>("/file_player_start", 1, boost::bind(&ROSThread::FilePlayerStart, this, _1));
  stop_sub_    = nh_.subscribe<std_msgs::Bool>("/file_player_stop", 1, boost::bind(&ROSThread::FilePlayerStop, this, _1));

  altimeter_pub_ = nh_.advertise<irp_sen_msgs::altimeter>("/altimeter_data", 1000);
  encoder_pub_ = nh_.advertise<irp_sen_msgs::encoder>("/encoder_count", 1000);
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1000);
  fog_pub_ = nh_.advertise<irp_sen_msgs::fog_3axis>("/dsp1760_data", 1000);
  gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
  vrs_pub_ = nh_.advertise<irp_sen_msgs::vrs>("/vrs_gps_data", 1000);
  vrs_gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/vrs_gps/fix", 1000);
  imu_origin_pub_ = nh_.advertise<irp_sen_msgs::imu>("/xsens_imu_data", 1000);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);
  imu_correct_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu_correct", 1000);
  magnet_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag", 1000);
  velodyne_left_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ns2/velodyne_points", 1000);
  velodyne_right_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ns1/velodyne_points", 1000);
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
}

void ROSThread::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void ROSThread::Ready()
{

  data_stamp_thread_.active_ = false;
  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();
  altimter_thread_.active_ = false;
  altimter_thread_.cv_.notify_all();
  if(altimter_thread_.thread_.joinable()) altimter_thread_.thread_.join();
  encoder_thread_.active_ = false;
  encoder_thread_.cv_.notify_all();
  if(encoder_thread_.thread_.joinable()) encoder_thread_.thread_.join();
  fog_thread_.active_ = false;
  fog_thread_.cv_.notify_all();
  if(fog_thread_.thread_.joinable()) fog_thread_.thread_.join();
  gps_thread_.active_ = false;
  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();
  vrs_thread_.active_ = false;
  vrs_thread_.cv_.notify_all();
  if(vrs_thread_.thread_.joinable()) vrs_thread_.thread_.join();
  imu_thread_.active_ = false;
  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();
  velodyne_left_thread_.active_ = false;
  velodyne_left_thread_.cv_.notify_all();
  if(velodyne_left_thread_.thread_.joinable()) velodyne_left_thread_.thread_.join();
  velodyne_right_thread_.active_ = false;
  velodyne_right_thread_.cv_.notify_all();
  if(velodyne_right_thread_.thread_.joinable()) velodyne_right_thread_.thread_.join();


  ifstream f((data_folder_path_+"/sensor_data/data_stamp.csv").c_str());
  if(!f.good()){
     cout << "Please check file path. Input path is wrong" << endl;
     return;
  }
  f.close();

  //Read CSV file and make map
  FILE *fp;
  int64_t stamp;
  //data stamp data load

  fp = fopen((data_folder_path_+"/sensor_data/data_stamp.csv").c_str(),"r");
  char data_name[50];
  data_stamp_.clear();
  while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
//    data_stamp_[stamp] = data_name;
    data_stamp_.insert( multimap<int64_t, string>::value_type(stamp, data_name));
  }
  cout << "Stamp data are loaded" << endl;
  fclose(fp);

  initial_data_stamp_ = data_stamp_.begin()->first - 1;
  last_data_stamp_ = prev(data_stamp_.end(),1)->first - 1;

  //Read altimeter data
  fp = fopen((data_folder_path_+"/sensor_data/altimeter.csv").c_str(),"r");
  double altimeter_value;
  irp_sen_msgs::altimeter altimeter_data;
  altimeter_data_.clear();
  while(fscanf(fp,"%ld,%lf\n",&stamp,&altimeter_value) == 2){
    altimeter_data.header.stamp.fromNSec(stamp);
    altimeter_data.header.frame_id = "altimeter";
    altimeter_data.data = altimeter_value;
    altimeter_data_[stamp] = altimeter_data;
  }
  cout << "Altimeter data are loaded" << endl;
  fclose(fp);

  //Read encoder data

  //Read encoder calibration data
  std::ifstream file((data_folder_path_+"/calibration/EncoderParameter.txt").c_str());
   std::string str;
   while (std::getline(file, str))
   {
     cout << str << endl;
     vector<string> strs;
     boost::split(strs, str, boost::is_any_of(" "));
     if(!strs[1].compare("resolution:")){
//       cout << strs[2] << endl;
       encoder_resolution_ = std::stoi(strs[2]);
     }
     if(!strs[1].compare("left")){
//       cout << strs[4] << endl;
       encoder_left_diameter_ = std::stod(strs[4]);
     }
     if(!strs[1].compare("right")){
//       cout << strs[4] << endl;
       encoder_right_diameter_ = std::stod(strs[4]);
     }
     if(!strs[1].compare("wheel")){
//       cout << strs[3] << endl;
       encoder_wheel_base_ = std::stod(strs[3]);
     }
   }
   if(encoder_resolution_ != 0){
      encoder_param_load_flag_ = true;
   }

  //Read encode data

  fp = fopen((data_folder_path_+"/sensor_data/encoder.csv").c_str(),"r");
  int64_t pre_left_count = 0, pre_right_count = 0;
  int64_t left_count, right_count;
  int64_t stop_start_stamp, stop_end_stamp;
  bool stop_period_start = false;
  irp_sen_msgs::encoder encoder_data;
  encoder_data_.clear();
  odometry_data_.clear();
  int encoder_stop_count = 0;
  int64_t current_stamp = 0, prev_stamp = 0;
  while(fscanf(fp,"%ld,%ld,%ld\n",&stamp,&left_count,&right_count) == 3){
    if(left_count == pre_left_count && right_count == pre_right_count && stop_period_start == false){
      encoder_stop_count++;
      if(encoder_stop_count >= 10){
//        cout << "start: " << left_count << " " << right_count << endl;
        stop_start_stamp = stamp;
        stop_period_start = true;
        encoder_stop_count = 0;
      }
    }else{
      encoder_stop_count = 0;
    }
    if((left_count != pre_left_count || right_count != pre_right_count)&& stop_period_start == true){
//      cout << "end: " << left_count << " " << right_count << endl;
      stop_end_stamp = stamp;
      stop_period_[stop_start_stamp] = stop_end_stamp;
      stop_period_start = false;
    }

    encoder_data.header.stamp.fromNSec(stamp);
    encoder_data.header.frame_id = "encoder";
    encoder_data.left_count = left_count;
    encoder_data.right_count = right_count;
    encoder_data_[stamp] = encoder_data;

    if(prev_stamp == 0){
      prev_stamp = stamp - 10000000;
    }
    if(pre_left_count == 0 && pre_right_count == 0){
      pre_left_count = left_count;
      pre_right_count = right_count;
    }
    current_stamp = stamp;

    //calculate odometry

    if(encoder_param_load_flag_){
      int64_t d_left_cnt = left_count - pre_left_count;
      int64_t d_right_cnt = right_count - pre_right_count;

      double left_distnace = ((double)d_left_cnt/(double)encoder_resolution_)*encoder_left_diameter_*M_PI;
      double right_distance = ((double)d_right_cnt/(double)encoder_resolution_)*encoder_right_diameter_*M_PI;
      double stamp_diff = static_cast<double>(current_stamp - prev_stamp);
      double dx = (left_distnace + right_distance)*0.5;
      double dy = 0.0;
      double dtheta = (right_distance - left_distnace)/encoder_wheel_base_;
      double vx = dx/stamp_diff;
      double vy = dy/stamp_diff;
      double vth = dtheta/stamp_diff;

      double delta_x = (dx * cos(encoder_theta_));
      double delta_y = (dx * sin(encoder_theta_));
      double delta_th = dtheta;

      encoder_x_ += delta_x;
      encoder_y_ += delta_y;
      encoder_theta_ += delta_th;
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(encoder_theta_);
      //Odometry message
      nav_msgs::Odometry odom;
      odom.header.stamp.fromNSec(current_stamp);
      odom.header.frame_id = "map";

      //set the position
      odom.pose.pose.position.x = encoder_x_;
      odom.pose.pose.position.y = encoder_y_;;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //pose covariance (6x6)
      odom.pose.covariance[0] = 1;
      odom.pose.covariance[7] = 1;
      odom.pose.covariance[14] = 1;
      odom.pose.covariance[21] = 1;
      odom.pose.covariance[28] = 1;
      odom.pose.covariance[35] = 1;
      //twist covariance(6x6)
      odom.twist.covariance[0] = 1;
      odom.twist.covariance[7] = 1;
      odom.twist.covariance[14] = 1;
      odom.twist.covariance[21] = 1;
      odom.twist.covariance[28] = 1;
      odom.twist.covariance[35] = 1;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;

      odometry_data_[stamp] = odom;

    }
    prev_stamp = current_stamp;
    pre_left_count = left_count;
    pre_right_count = right_count;

  }

//  cout << stop_period_.size() << endl;
  cout << "Encoder data are loaded" << endl;
  fclose(fp);

  //Read fog data
  fp = fopen((data_folder_path_+"/sensor_data/fog.csv").c_str(),"r");
  float d_roll, d_pitch, d_yaw;
  irp_sen_msgs::fog_3axis fog_data;
  fog_data_.clear();
  while(fscanf(fp,"%ld,%f,%f,%f\n",&stamp,&d_roll,&d_pitch,&d_yaw) == 4){
    fog_data.header.stamp.fromNSec(stamp);
    fog_data.header.frame_id = "dsp1760";
    fog_data.d_roll = d_roll;
    fog_data.d_pitch = d_pitch;
    fog_data.d_yaw = d_yaw;
    fog_data_[stamp] = fog_data;
  }
  cout << "Fog data are loaded" << endl;
  fclose(fp);

  //Read gps data
  fp = fopen((data_folder_path_+"/sensor_data/gps.csv").c_str(),"r");
  double latitude, longitude, altitude, altitude_orthometric;
  double cov[9];
  sensor_msgs::NavSatFix gps_data;
  gps_data_.clear();
  while(fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&latitude,&longitude,&altitude,&cov[0],&cov[1],&cov[2],&cov[3],&cov[4],&cov[5],&cov[6],&cov[7],&cov[8]) == 13){
    gps_data.header.stamp.fromNSec(stamp);
    gps_data.header.frame_id = "navsat_link";
    gps_data.latitude = latitude;
    gps_data.longitude = longitude;
    gps_data.altitude = altitude;
    for(int i = 0 ; i < 9 ; i ++) gps_data.position_covariance[i] = cov[i];
    gps_data.position_covariance_type = 1;
    gps_data_[stamp] = gps_data;



  }
  cout << "Gps data are loaded" << endl;
  fclose(fp);

  //Read gps data
  fp = fopen((data_folder_path_+"/sensor_data/vrs_gps.csv").c_str(),"r");
  double x_coordinate, y_coordinate, horizental_precision, lat_std, lon_std, altitude_std, heading_magnet, speed_knot, speed_km;
  int fix_state, number_of_sat, heading_valid;
  char GNVTG_mode;
  irp_sen_msgs::vrs vrs_data;
  sensor_msgs::NavSatFix vrs_gps_data;
  vrs_data_.clear();
  vrs_gps_data_.clear();
//  gps_odometry_data_.clear();
  while(1){
    int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%d,%d,%lf,%lf,%lf,%lf,%d,%lf,%lf,%lf,%c,%lf\n",&stamp,&latitude,&longitude,&x_coordinate,
               &y_coordinate,&altitude,&fix_state,&number_of_sat,&horizental_precision,&lat_std,&lon_std,&altitude_std,
               &heading_valid,&heading_magnet,&speed_knot,&speed_km,&GNVTG_mode,&altitude_orthometric);
    if(length == 18){

        vrs_data.header.stamp.fromNSec(stamp);
        vrs_data.header.frame_id = "vrs_gps";
        vrs_data.latitude = latitude;
        vrs_data.altitude_orthometric = altitude_orthometric;
        vrs_data.longitude = longitude;
        vrs_data.x_coordinate = x_coordinate;
        vrs_data.y_coordinate = y_coordinate;
        vrs_data.altitude = altitude;
        vrs_data.fix_state = fix_state;
        if(fix_state == 1) vrs_data.fix_state_str = "normal";
        if(fix_state == 4) vrs_data.fix_state_str = "fix";
        if(fix_state == 5) vrs_data.fix_state_str = "float";
        vrs_data.number_of_sat = number_of_sat;
        vrs_data.horizental_precision = horizental_precision;
        vrs_data.lat_std = lat_std;
        vrs_data.lon_std = lon_std;
        vrs_data.altitude_std = altitude_std;
        vrs_data.heading_valid = heading_valid;
        vrs_data.heading_magnet = heading_magnet;
        vrs_data.speed_knot = speed_knot;
        vrs_data.speed_km = speed_km;
        vrs_data.GNVTG_mode = GNVTG_mode;
        vrs_data_[stamp] = vrs_data;
        
        vrs_gps_data.header.stamp.fromNSec(stamp);
        vrs_gps_data.header.frame_id = "navsat_link";
    		vrs_gps_data.latitude = latitude;
    		vrs_gps_data.longitude = longitude;
    		vrs_gps_data.altitude = altitude;
    		vrs_gps_data.position_covariance[0] = 1.0;
    		vrs_gps_data.position_covariance[1] = 0.0;
    		vrs_gps_data.position_covariance[2] = 0.0;
    		vrs_gps_data.position_covariance[3] = 0.0;
    		vrs_gps_data.position_covariance[4] = 1.0;
    		vrs_gps_data.position_covariance[5] = 0.0;
    		vrs_gps_data.position_covariance[6] = 0.0;
    		vrs_gps_data.position_covariance[7] = 0.0;
    		vrs_gps_data.position_covariance[8] = 1.0;
    		vrs_gps_data.position_covariance_type = 1;
    		vrs_gps_data_[stamp] = vrs_gps_data;

    }else if(length == 17){
        vrs_data.header.stamp.fromNSec(stamp);
        vrs_data.header.frame_id = "vrs_gps";
        vrs_data.latitude = latitude;
        vrs_data.longitude = longitude;
        vrs_data.x_coordinate = x_coordinate;
        vrs_data.y_coordinate = y_coordinate;
        vrs_data.altitude = altitude;
        vrs_data.fix_state = fix_state;
        if(fix_state == 1) vrs_data.fix_state_str = "normal";
        if(fix_state == 4) vrs_data.fix_state_str = "fix";
        if(fix_state == 5) vrs_data.fix_state_str = "float";
        vrs_data.number_of_sat = number_of_sat;
        vrs_data.horizental_precision = horizental_precision;
        vrs_data.lat_std = lat_std;
        vrs_data.lon_std = lon_std;
        vrs_data.altitude_std = altitude_std;
        vrs_data.heading_valid = heading_valid;
        vrs_data.heading_magnet = heading_magnet;
        vrs_data.speed_knot = speed_knot;
        vrs_data.speed_km = speed_km;
        vrs_data.GNVTG_mode = GNVTG_mode;
        vrs_data_[stamp] = vrs_data;
        
        vrs_gps_data.header.stamp.fromNSec(stamp);
        vrs_gps_data.header.frame_id = "navsat_link";
    		vrs_gps_data.latitude = latitude;
    		vrs_gps_data.longitude = longitude;
    		vrs_gps_data.altitude = altitude;
    		vrs_gps_data.position_covariance[0] = 1.0;
    		vrs_gps_data.position_covariance[1] = 0.0;
    		vrs_gps_data.position_covariance[2] = 0.0;
    		vrs_gps_data.position_covariance[3] = 0.0;
    		vrs_gps_data.position_covariance[4] = 1.0;
    		vrs_gps_data.position_covariance[5] = 0.0;
    		vrs_gps_data.position_covariance[6] = 0.0;
    		vrs_gps_data.position_covariance[7] = 0.0;
    		vrs_gps_data.position_covariance[8] = 1.0;
    		vrs_gps_data.position_covariance_type = 1;
    		vrs_gps_data_[stamp] = vrs_gps_data;

    }else{
        break;

    }
  }
  cout << "Vrs gps data are loaded" << endl;
  fclose(fp);

  //Read IMU data
  fp = fopen((data_folder_path_+"/sensor_data/xsens_imu.csv").c_str(),"r");
  double q_x,q_y,q_z,q_w,x,y,z,g_x,g_y,g_z,a_x,a_y,a_z,m_x,m_y,m_z;
  irp_sen_msgs::imu imu_data_origin;
  sensor_msgs::Imu imu_data;
  sensor_msgs::MagneticField mag_data;
  imu_data_.clear();
  mag_data_.clear();

  while(1){
    int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z);
    if(length != 8 && length != 17) break;
    if(length == 8){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "base_link";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;

      imu_data_[stamp] = imu_data;
      imu_data_version_ = 1;


      imu_data_origin.header.stamp.fromNSec(stamp);
      imu_data_origin.header.frame_id = "base_link";
      imu_data_origin.quaternion_data.x = q_x;
      imu_data_origin.quaternion_data.y = q_y;
      imu_data_origin.quaternion_data.z = q_z;
      imu_data_origin.quaternion_data.w = q_w;
      imu_data_origin.eular_data.x = x;
      imu_data_origin.eular_data.y = y;
      imu_data_origin.eular_data.z = z;
      imu_data_origin_[stamp] = imu_data_origin;


    }else if(length == 17){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "base_link";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;
      imu_data.angular_velocity.x = g_x;
      imu_data.angular_velocity.y = g_y;
      imu_data.angular_velocity.z = g_z;
      imu_data.linear_acceleration.x = a_x;
      imu_data.linear_acceleration.y = a_y;
      imu_data.linear_acceleration.z = a_z;

      imu_data.orientation_covariance[0] = 3;
      imu_data.orientation_covariance[4] = 3;
      imu_data.orientation_covariance[8] = 3;
      imu_data.angular_velocity_covariance[0] = 3;
      imu_data.angular_velocity_covariance[4] = 3;
      imu_data.angular_velocity_covariance[8] = 3;
      imu_data.linear_acceleration_covariance[0] = 3;
      imu_data.linear_acceleration_covariance[4] = 3;
      imu_data.linear_acceleration_covariance[8] = 3;


      imu_data_[stamp] = imu_data;
      mag_data.header.stamp.fromNSec(stamp);
      mag_data.header.frame_id = "base_link";
      mag_data.magnetic_field.x = m_x;
      mag_data.magnetic_field.y = m_y;
      mag_data.magnetic_field.z = m_z;
      mag_data_[stamp] = mag_data;
      imu_data_version_ = 2;


      imu_data_origin.header.stamp.fromNSec(stamp);
      imu_data_origin.header.frame_id = "base_link";
      imu_data_origin.quaternion_data.x = q_x;
      imu_data_origin.quaternion_data.y = q_y;
      imu_data_origin.quaternion_data.z = q_z;
      imu_data_origin.quaternion_data.w = q_w;
      imu_data_origin.eular_data.x = x;
      imu_data_origin.eular_data.y = y;
      imu_data_origin.eular_data.z = z;
      imu_data_origin.gyro_data.x = g_x;
      imu_data_origin.gyro_data.y = g_y;
      imu_data_origin.gyro_data.z = g_z;
      imu_data_origin.acceleration_data.x = a_x;
      imu_data_origin.acceleration_data.y = a_y;
      imu_data_origin.acceleration_data.z = a_z;
      imu_data_origin.magneticfield_data.x = m_x;
      imu_data_origin.magneticfield_data.y = m_y;
      imu_data_origin.magneticfield_data.z = m_z;
      imu_data_origin_[stamp] = imu_data_origin;

    }
  }
  cout << "IMU data are loaded" << endl;
  fclose(fp);

  velodyne_left_file_list_.clear();
  velodyne_right_file_list_.clear();


  GetDirList(data_folder_path_ + "/sensor_data/VLP_left",velodyne_left_file_list_);
  GetDirList(data_folder_path_ + "/sensor_data/VLP_right",velodyne_right_file_list_);

  data_stamp_thread_.active_ = true;
  altimter_thread_.active_ = true;
  encoder_thread_.active_ = true;
  fog_thread_.active_ = true;
  gps_thread_.active_ = true;
  vrs_thread_.active_ = true;
  imu_thread_.active_ = true;
  velodyne_left_thread_.active_ = true;
  velodyne_right_thread_.active_ = true;

  data_stamp_thread_.thread_ = std::thread(&ROSThread::DataStampThread,this);
  altimter_thread_.thread_ = std::thread(&ROSThread::AltimeterThread,this);
  encoder_thread_.thread_ = std::thread(&ROSThread::EncoderThread,this);
  fog_thread_.thread_ = std::thread(&ROSThread::FogThread,this);
  gps_thread_.thread_ = std::thread(&ROSThread::GpsThread,this);
  vrs_thread_.thread_ = std::thread(&ROSThread::VrsThread,this);
  imu_thread_.thread_ = std::thread(&ROSThread::ImuThread,this);
  velodyne_left_thread_.thread_ = std::thread(&ROSThread::VelodyneLeftThread,this);
  velodyne_right_thread_.thread_ = std::thread(&ROSThread::VelodyneRightThread,this);
}

void ROSThread::DataStampThread()
{
  auto stop_region_iter = stop_period_.begin();

  for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++){
    auto stamp = iter->first;



    while((stamp > (initial_data_stamp_+processed_stamp_))&&(data_stamp_thread_.active_ == true)){
      if(processed_stamp_ == 0){
          iter = data_stamp_.begin();
          stop_region_iter = stop_period_.begin();
          stamp = iter->first;
      }
      usleep(1);
      if(reset_process_stamp_flag_ == true) break;
      //wait for data publish
    }

    if(reset_process_stamp_flag_ == true){
      auto target_stamp = processed_stamp_ + initial_data_stamp_;
      //set iter
      iter = data_stamp_.lower_bound(target_stamp);
      iter = prev(iter,1);
      //set stop region order
      auto new_stamp = iter->first;
      stop_region_iter = stop_period_.upper_bound(new_stamp);

      reset_process_stamp_flag_ = false;
      continue;
    }


    //check whether stop region or not
    if(stamp == stop_region_iter->first){
      if(stop_skip_flag_ == true){
        cout << "Skip stop section!!" << endl;
        iter = data_stamp_.find(stop_region_iter->second);  //find stop region end
        iter = prev(iter,1);
        processed_stamp_ = stop_region_iter->second - initial_data_stamp_;
      }
      stop_region_iter++;
      if(stop_skip_flag_ == true){
        continue;
      }
    }

    if(data_stamp_thread_.active_ == false) return;
    if(iter->second.compare("altimeter") == 0){
      altimter_thread_.push(stamp);
      altimter_thread_.cv_.notify_all();
    }else if(iter->second.compare("encoder") == 0){
      encoder_thread_.push(stamp);
      encoder_thread_.cv_.notify_all();
    }else if(iter->second.compare("fog") == 0){
      fog_thread_.push(stamp);
      fog_thread_.cv_.notify_all();
    }else if(iter->second.compare("gps") == 0){
      gps_thread_.push(stamp);
      gps_thread_.cv_.notify_all();
    }else if(iter->second.compare("vrs") == 0){
      vrs_thread_.push(stamp);
      vrs_thread_.cv_.notify_all();
    }else if(iter->second.compare("imu") == 0){
      imu_thread_.push(stamp);
      imu_thread_.cv_.notify_all();
    }else if(iter->second.compare("velodyne_left") == 0){
        velodyne_left_thread_.push(stamp);
        velodyne_left_thread_.cv_.notify_all();
    }else if(iter->second.compare("velodyne_right") == 0){
        velodyne_right_thread_.push(stamp);
        velodyne_right_thread_.cv_.notify_all();
    }

    stamp_show_count_++;
    if(stamp_show_count_ > 100){
      stamp_show_count_ = 0;
      emit StampShow(stamp);
    }

    if(prev_clock_stamp_ == 0 || (stamp - prev_clock_stamp_) > 10000000){
        rosgraph_msgs::Clock clock;
        clock.clock.fromNSec(stamp);
        clock_pub_.publish(clock);
        prev_clock_stamp_ = stamp;
    }

    if(loop_flag_ == true && iter == prev(data_stamp_.end(),1)){
        iter = data_stamp_.begin();
        stop_region_iter = stop_period_.begin();
        processed_stamp_ = 0;
    }
    if(loop_flag_ == false && iter == prev(data_stamp_.end(),1)){
        play_flag_ = false;
        while(!play_flag_){
            iter = data_stamp_.begin();
            stop_region_iter = stop_period_.begin();
            processed_stamp_ = 0;
            usleep(10000);
        }
    }


  }
  cout << "Data publish complete" << endl;

}

void ROSThread::AltimeterThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(altimter_thread_.mutex_);
    altimter_thread_.cv_.wait(ul);
    if(altimter_thread_.active_ == false) return;
    ul.unlock();

    while(!altimter_thread_.data_queue_.empty()){
      auto data = altimter_thread_.pop();
      //process
      if(altimeter_data_.find(data) != altimeter_data_.end()){
        altimeter_pub_.publish(altimeter_data_[data]);
      }
    }
    if(altimter_thread_.active_ == false) return;
  }
}

void ROSThread::EncoderThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(encoder_thread_.mutex_);
    encoder_thread_.cv_.wait(ul);
    if(encoder_thread_.active_ == false) return;
    ul.unlock();

    while(!encoder_thread_.data_queue_.empty()){
      auto data = encoder_thread_.pop();
      //process
      if(encoder_data_.find(data) != encoder_data_.end()){
        encoder_pub_.publish(encoder_data_[data]);
        if(encoder_param_load_flag_){
          odometry_pub_.publish(odometry_data_[data]);
        }
      }

    }
    if(encoder_thread_.active_ == false) return;
  }
}

void ROSThread::FogThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(fog_thread_.mutex_);
    fog_thread_.cv_.wait(ul);
    if(fog_thread_.active_ == false) return;
    ul.unlock();

    while(!fog_thread_.data_queue_.empty()){
      auto data = fog_thread_.pop();
      //process
      if(fog_data_.find(data) != fog_data_.end()){
        fog_pub_.publish(fog_data_[data]);
      }

    }
    if(fog_thread_.active_ == false) return;
  }
}

void ROSThread::GpsThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(gps_thread_.mutex_);
    gps_thread_.cv_.wait(ul);
    if(gps_thread_.active_ == false) return;
    ul.unlock();

    while(!gps_thread_.data_queue_.empty()){
      auto data = gps_thread_.pop();
      //process
      if(gps_data_.find(data) != gps_data_.end()){
        gps_pub_.publish(gps_data_[data]);
      }

    }
    if(gps_thread_.active_ == false) return;
  }
}

void ROSThread::VrsThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(vrs_thread_.mutex_);
    vrs_thread_.cv_.wait(ul);
    if(vrs_thread_.active_ == false) return;
    ul.unlock();

    while(!vrs_thread_.data_queue_.empty()){
      auto data = vrs_thread_.pop();
      //process
      if(vrs_data_.find(data) != vrs_data_.end()){
        vrs_pub_.publish(vrs_data_[data]);
        vrs_gps_pub_.publish(vrs_gps_data_[data]);
//        gps_odometry_pub_.publish(gps_odometry_data_[data]);
      }

    }
    if(vrs_thread_.active_ == false) return;
  }
}

void ROSThread::ImuThread() {

    while(1){

        std::unique_lock<std::mutex> ul(imu_thread_.mutex_);
        imu_thread_.cv_.wait(ul);

        if (imu_thread_.active_ == false)
            return;

        ul.unlock();

        while(!imu_thread_.data_queue_.empty()) {

            auto data = imu_thread_.pop();
            //process
            if(imu_data_.find(data) != imu_data_.end()){

            imu_pub_.publish(imu_data_[data]);
            imu_correct_pub_.publish(imu_data_[data]);
            imu_origin_pub_.publish(imu_data_origin_[data]);

            if(imu_data_version_ == 2)
                magnet_pub_.publish(mag_data_[data]);
            }
        }

    if(imu_thread_.active_ == false)
        return;
    }
}

void ROSThread::TimerCallback(const ros::TimerEvent&)
{
    int64_t current_stamp = ros::Time::now().toNSec();
    if(play_flag_ == true && pause_flag_ == false){
      processed_stamp_ += static_cast<int64_t>(static_cast<double>(current_stamp - pre_timer_stamp_) * play_rate_);
    }
    pre_timer_stamp_ = current_stamp;

    if(play_flag_ == false){
      processed_stamp_ = 0; //reset
      prev_clock_stamp_ = 0;
    }
}
void ROSThread::VelodyneLeftThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(velodyne_left_thread_.mutex_);
    velodyne_left_thread_.cv_.wait(ul);
    if(velodyne_left_thread_.active_ == false) return;
    ul.unlock();

    while(!velodyne_left_thread_.data_queue_.empty()){
      auto data = velodyne_left_thread_.pop();

      //publish data
      if(to_string(data) + ".bin" == velodyne_left_next_.first){
        //publish
        velodyne_left_next_.second.header.stamp.fromNSec(data);
        velodyne_left_next_.second.header.frame_id = "left_velodyne";
        velodyne_left_pub_.publish(velodyne_left_next_.second);

      }else{
//        cout << "Re-load left velodyne from path" << endl;
        //load current data
        pcl::PointCloud<PointXYZIR> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/sensor_data/VLP_left" +"/"+ to_string(data) + ".bin";
        if(find(next(velodyne_left_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_left_file_list_.end(),to_string(data)+".bin") != velodyne_left_file_list_.end()){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);

            float angle;
            uint16_t ring;

            while(!file.eof()){
                PointXYZIR point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                angle = atan2(point.z, sqrt(point.x*point.x+point.y*point.y)) / M_PI * 180 + 15;
                ring = round(angle / 2);
                point.ring = ring;
                cloud.points.push_back (point);
            }
            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp.fromNSec(data);
            publish_cloud.header.frame_id = "left_velodyne";
            velodyne_left_pub_.publish(publish_cloud);

        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<PointXYZIR> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      current_file_index = find(next(velodyne_left_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_left_file_list_.end(),to_string(data)+".bin") - velodyne_left_file_list_.begin();
      if(find(next(velodyne_left_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_left_file_list_.end(),velodyne_left_file_list_[current_file_index+1]) != velodyne_left_file_list_.end()){
          string next_file_name = data_folder_path_ + "/sensor_data/VLP_left" +"/"+ velodyne_left_file_list_[current_file_index+1];

          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);

          float angle;
          uint16_t ring;

          while(!file.eof()){
              PointXYZIR point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));

              angle = atan2(point.z, sqrt(point.x*point.x+point.y*point.y)) / M_PI * 180 + 15;
              ring = round(angle / 2);
              point.ring = ring;

              cloud.points.push_back (point);
          }
          file.close();
          pcl::toROSMsg(cloud, publish_cloud);
          velodyne_left_next_ = make_pair(velodyne_left_file_list_[current_file_index+1], publish_cloud);
      }
      previous_file_index = current_file_index;
    }
    if(velodyne_left_thread_.active_ == false) return;
  }
}
void ROSThread::VelodyneRightThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(velodyne_right_thread_.mutex_);
    velodyne_right_thread_.cv_.wait(ul);
    if(velodyne_right_thread_.active_ == false) return;
    ul.unlock();

    while(!velodyne_right_thread_.data_queue_.empty()){
      auto data = velodyne_right_thread_.pop();
      //process

      //publish data
      if(to_string(data) + ".bin" == velodyne_right_next_.first){
        //publish
        velodyne_right_next_.second.header.stamp.fromNSec(data);
        velodyne_right_next_.second.header.frame_id = "right_velodyne";
        velodyne_right_pub_.publish(velodyne_right_next_.second);

      }else{
//        cout << "Re-load right velodyne from path" << endl;
        //load current data
        pcl::PointCloud<PointXYZIR> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/sensor_data/VLP_right" +"/"+ to_string(data) + ".bin";
        if(find(next(velodyne_right_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_right_file_list_.end(),to_string(data)+".bin") != velodyne_right_file_list_.end()){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);

            float angle;
            uint16_t ring;

            while(!file.eof()){
                PointXYZIR point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));

                angle = atan2(point.z, sqrt(point.x*point.x+point.y*point.y)) / M_PI * 180 + 15;
                ring = round(angle / 2);
                point.ring = ring  + 16;

                //file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
                cloud.points.push_back (point);
            }
            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp.fromNSec(data);
            publish_cloud.header.frame_id = "right_velodyne";
            velodyne_right_pub_.publish(publish_cloud);

        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<PointXYZIR> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      current_file_index = find(next(velodyne_right_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_right_file_list_.end(),to_string(data)+".bin") - velodyne_right_file_list_.begin();
      if(find(next(velodyne_right_file_list_.begin(),max(0,previous_file_index-search_bound_)),velodyne_right_file_list_.end(),velodyne_right_file_list_[current_file_index+1]) != velodyne_right_file_list_.end()){
          string next_file_name = data_folder_path_ + "/sensor_data/VLP_right" +"/"+ velodyne_right_file_list_[current_file_index+1];

          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);

          float angle;
          uint16_t ring;

          while(!file.eof()){
              PointXYZIR point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));

              angle = atan2(point.z, sqrt(point.x*point.x+point.y*point.y)) / M_PI * 180 + 15;
              ring = round(angle / 2);
              point.ring = ring + 16;

              //file.read(reinterpret_cast<char *>(&point.ring), sizeof(uint16_t));
              cloud.points.push_back (point);
          }
          file.close();
          pcl::toROSMsg(cloud, publish_cloud);
          velodyne_right_next_ = make_pair(velodyne_right_file_list_[current_file_index+1], publish_cloud);
      }

      previous_file_index = current_file_index;
    }
    if(velodyne_right_thread_.active_ == false) return;
  }
}

int ROSThread::GetDirList(string dir, vector<string> &files)
{

  vector<string> tmp_files;
  struct dirent **namelist;
  int n;
  n = scandir(dir.c_str(),&namelist, 0 , alphasort);
  if (n < 0)
      perror("scandir");
  else {
      while (n--) {
      if(string(namelist[n]->d_name) != "." && string(namelist[n]->d_name) != ".."){
        tmp_files.push_back(string(namelist[n]->d_name));
      }
      free(namelist[n]);
      }
      free(namelist);
  }

  for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++){
    files.push_back(*iter);
  }
    return 0;
}

void ROSThread::FilePlayerStart(const std_msgs::BoolConstPtr& msg)
{
  if(auto_start_flag_ == true){
    cout << "File player auto start" << endl;
    usleep(1000000);
    play_flag_ = false;
    emit StartSignal();
  }
}

void ROSThread::FilePlayerStop(const std_msgs::BoolConstPtr& msg)
{
  cout << "File player auto stop" << endl;
  play_flag_ = true;
  emit StartSignal();
}
void ROSThread::ResetProcessStamp(int position){

  if(position > 0 && position < 10000){

    processed_stamp_ = static_cast<int64_t>(static_cast<float>(last_data_stamp_ - initial_data_stamp_)*static_cast<float>(position)/static_cast<float>(10000));
    reset_process_stamp_flag_ = true;
  }
}
