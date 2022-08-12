#include "localization_wrapper.h"

#include <iomanip>
#include <cmath>

#include <glog/logging.h>

#include "imu_gps_localizer/base_type.h"
#include "imu_gps_localizer/gps_processor.h"

// #include "imu_gps_localizer/utils.h"
#include <GeographicLib/LocalCartesian.hpp>

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);

    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    const Eigen::Vector3d I_p_Gps(x, y, z);

    std::string log_folder = "/home";
    ros::param::get("log_folder", log_folder);

    // Log.
    // file_state_.open(log_folder + "/state.csv");
    // file_gps_.open(log_folder +"/gps.csv");

    file_state_.open(log_folder + "/uppc_imu.txt");
    file_gps_.open(log_folder + "/uppc.txt");

    last_state_posetion_.setZero();


    // Initialization imu gps localizer.
    imu_gps_localizer_ptr_ = 
        std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                              acc_bias_noise, gyro_bias_noise,
                                                              I_p_Gps);

    // Subscribe topics.
    imu_sub_ = nh.subscribe("/android/imu", 10,  &LocalizationWrapper::ImuCallback, this);
    gps_position_sub_ = nh.subscribe("/android/fix", 10,  &LocalizationWrapper::GpsPositionCallback, this);

    state_pub_ = nh.advertise<nav_msgs::Path>("fused_path", 10);
    gps_path_pub_ = nh.advertise<nav_msgs::Path>("gps_path", 10);
    gps_noise_pub_ = nh.advertise<nav_msgs::Path>("gps_noise_path", 10);

}

LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    file_gps_.close();
}

void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;


    // double delta_time_gps_imu;
    // delta_time_gps_imu = abs(imu_data_ptr->timestamp - gps_timestamp_);

    // std::cout << "--------------------------------" << std::endl;
    // std::cout << std::fixed << std::setprecision(6) << "delta_time_gps_imu: " << delta_time_gps_imu << std::endl;

    // if(delta_time_gps_imu > 1 && delta_time_gps_imu < 3)
    // {
    //     return;
    // }
    
    ImuGpsLocalization::State fused_state;
    const bool ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state, gps_timestamp_);
    if (!ok) {
        return;
    }

    // Publish fused state.
    ConvertStateToRosTopic(fused_state);
    state_pub_.publish(ros_path_);

    // Log fused state.
    LogState(fused_state);
}

void LocalizationWrapper::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
    // Check the gps_status.
    // if (gps_msg_ptr->status.status != 2) {
    //     LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
    //     return;
    // }

    
    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->latitude,
                         gps_msg_ptr->longitude,
                         gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());


    // std::cout << "---------------------" << std::endl;
    // std::cout << gps_data_ptr->cov(0,0) << ", " << gps_data_ptr->cov(1,1) << std::endl;
    // std::cout << "---------------------" << std::endl;

    // if(gps_data_ptr->cov(0,0) > 100 || gps_data_ptr->cov(1,1) > 100)
    // {
    //     return;
    // }


    // if(!CheckGps(gps_data_ptr))
    // {
    //     return;
    // }

    gps_timestamp_ = gps_msg_ptr->header.stamp.toSec();


    ImuGpsLocalization::State fused_state;
    imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr, &fused_state);

    // ConvertStateToRosTopic(fused_state);
    // state_pub_.publish(ros_path_);

    // LogState(fused_state);

    LogGps(gps_data_ptr, fused_state);

    gps_path_pub_.publish(gps_path_);
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    Eigen::Quaterniond G_q_I(state.G_R_I);
    G_q_I.normalize();

    file_state_ << std::fixed << std::setprecision(6)
                << state.timestamp << " "
                << state.G_p_I[0] << " " << state.G_p_I[1] << " " << state.G_p_I[2] << " "
                << G_q_I.x() << " " << G_q_I.y() << " " << G_q_I.z() << " " << G_q_I.w() << std::endl;

    // Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    // pose.block<3,3>(0,0) = G_q_I.toRotationMatrix();
    // pose.block<3,1>(0,3) = Eigen::Vector3d(state.G_p_I[0], state.G_p_I[1], state.G_p_I[2]);
    // SavePose(file_state_, pose);

    // file_state_ << std::fixed << std::setprecision(15) << state.timestamp << " "
    //         << ekf_ptr_->state_ptr_->p_wb_[0] << " " << ekf_ptr_->state_ptr_->p_wb_[1] << " "
    //         << ekf_ptr_->state_ptr_->p_wb_[2] << " " << q_GI.x() << " " << q_GI.y() << " " << q_GI.z() << " "
    //         << q_GI.w() << std::endl;

}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data, const ImuGpsLocalization::State& state) {

    static GeographicLib::LocalCartesian local_cartesian;

    Eigen::Quaterniond G_q_I(state.G_R_I);
    G_q_I.normalize();


    if(!inited_)
    {
        inited_ = true;
        local_cartesian.Reset(gps_data->lla[0], gps_data->lla[1], gps_data->lla[2]);

    }
    // file_gps_ << std::fixed << std::setprecision(15)
    //           << gps_data->timestamp << ","
    //           << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    Eigen::Vector3d ned;
    
    local_cartesian.Forward(gps_data->lla[0], gps_data->lla[1], gps_data->lla[2], ned[0], ned[1], ned[2]);
    matrix.block<3,1>(0,3) = ned;
    // SavePose(file_gps_, matrix);

    file_gps_ << std::fixed << std::setprecision(6)
                << gps_data->timestamp << " "
                << ned[0] << " " << ned[1] << " " << ned[2] << " "
                << G_q_I.x() << " " << G_q_I.y() << " " << G_q_I.z() << " " << G_q_I.w() << std::endl;


    gps_path_.header.frame_id = "world";
    gps_path_.header.stamp = ros::Time::now();


    geometry_msgs::PoseStamped pose;
    pose.header = gps_path_.header;

    if(isnan(ned[0]) || isnan(ned[1]) || isnan(ned[2]))
    {
        // state.G_p_I[2] = 0;
        // std::cout << "nan" << std::endl;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
    }
    else
    {
        pose.pose.position.x = ned[0];
        pose.pose.position.y = ned[1];
        pose.pose.position.z = ned[2];
    }

    
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    gps_path_.poses.push_back(pose);


}


bool LocalizationWrapper::CheckGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data) {

    static GeographicLib::LocalCartesian local_cartesian;


    if(!first_)
    {
        first_ = true;
        local_cartesian.Reset(gps_data->lla[0], gps_data->lla[1], gps_data->lla[2]);
        last_gps_posetion_ = Eigen::Vector3d::Zero();

    }
    // file_gps_ << std::fixed << std::setprecision(15)
    //           << gps_data->timestamp << ","
    //           << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    Eigen::Vector3d ned;
    
    local_cartesian.Forward(gps_data->lla[0], gps_data->lla[1], gps_data->lla[2], ned[0], ned[1], ned[2]);

    curr_gps_posetion_ = ned;

    delta_x = abs(curr_gps_posetion_[0] - last_gps_posetion_[0]);
    delta_y = abs(curr_gps_posetion_[1] - last_gps_posetion_[1]);


    // std::cout << "delta_x: " << delta_x << std::endl;
    // std::cout << "delta_y: " << delta_y << std::endl;
    

    delta_x_.push_back(delta_x);
    delta_y_.push_back(delta_y);

    while(delta_x_.size() > 20 || delta_y_.size() > 20)
    {
        delta_x_.pop_front();
        delta_y_.pop_front();
    }

    // sort(delta_x_.begin(), delta_x_.end());
    // sort(delta_y_.begin(), delta_y_.end());


    // std::cout << "---------------------" << std::endl;
    // std::cout << "max delta_x: " << *std::max_element(delta_x_.begin(), delta_x_.end()) << std::endl;
    // std::cout << "max delta_y: " << *std::max_element(delta_y_.begin(), delta_y_.end()) << std::endl;
    // std::cout << "---------------------" << std::endl;

    double mean_delta_x, mean_delta_y, var_delta_x, var_delta_y;
    if(delta_x_.size() > 0)
    {
        mean_delta_x = std::accumulate(delta_x_.begin(), delta_x_.end(), 0.0) / delta_x_.size();
        mean_delta_y = std::accumulate(delta_y_.begin(), delta_y_.end(), 0.0) / delta_y_.size();
    }
    

    for(int i = 0; i < delta_x_.size(); i++)
    {
        var_delta_x += pow((delta_x_[i] - mean_delta_x), 2);
        var_delta_y += pow((delta_y_[i] - mean_delta_y), 2);
    }

    var_delta_x /= delta_x_.size();
    var_delta_y /= delta_y_.size();


    // std::cout << "--------------------------------" << std::endl;
    // std::cout << "var delta x: " << var_delta_x << std::endl;
    // std::cout << "var delta y: " << var_delta_y << std::endl;
    // std::cout << "--------------------------------" << std::endl;

    if(var_delta_x > 0.2 || var_delta_y > 0.2)
    {
        last_gps_posetion_ = curr_gps_posetion_;
        return false;
    }

    
    for(int i = 0; i < delta_x_.size(); i++)
    {
        
        // std::cout << "delta_x: " << delta_x_[i] << std::endl;
        // std::cout << "delta_y: " << delta_y_[i] << std::endl;

        if(delta_x_[i] > 2 || delta_y_[i] > 2)
        {
            last_gps_posetion_ = curr_gps_posetion_;
            return false;
        }
        
    }
    

    // if(delta_x > 5 || delta_y > 5)
    // {
    //     last_gps_posetion_ = curr_gps_posetion_;
    //     return false;
    // }

    last_gps_posetion_ = curr_gps_posetion_;

    return true;


}


void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;





    if(isnan(state.G_p_I[0]) || isnan(state.G_p_I[1]) || isnan(state.G_p_I[2]))
    {
        // state.G_p_I[2] = 0;
        std::cout << "nan" << std::endl;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
    }
    else
    {
        pose.pose.position.x = state.G_p_I[0];
        pose.pose.position.y = state.G_p_I[1];
        pose.pose.position.z = state.G_p_I[2];
    }


    curr_state_posetion_[0] = pose.pose.position.x;
    curr_state_posetion_[1] = pose.pose.position.y;
    curr_state_posetion_[2] = pose.pose.position.z;


    delta_x = abs(curr_state_posetion_[0] - last_state_posetion_[0]);
    delta_y = abs(curr_state_posetion_[1] - last_state_posetion_[1]);


    // std::cout << "state_delta_x: " << delta_x << std::endl;
    // std::cout << "state_delta_y: " << delta_y << std::endl;


    state_delta_x_.push_back(delta_x);
    state_delta_y_.push_back(delta_y);


    while(state_delta_x_.size() > 100 || state_delta_y_.size() > 100)
    {
        state_delta_x_.pop_front();
        state_delta_y_.pop_front();
    }

    sum_state_delta_x = std::accumulate(state_delta_x_.begin(), state_delta_x_.end(), 0.0);
    sum_state_delta_y = std::accumulate(state_delta_y_.begin(), state_delta_y_.end(), 0.0);


    // std::cout << "--------------------------------" << std::endl;
    // std::cout << "sum state delta x: " << sum_state_delta_x << std::endl;
    // std::cout << "sum state delta y: " << sum_state_delta_y << std::endl;
    // std::cout << "--------------------------------" << std::endl;


    // if(delta_x > 0.02 || delta_y > 0.02)
    // {
    //     pose.pose.position.x = last_state_posetion_[0];
    //     pose.pose.position.y = last_state_posetion_[1];
    //     last_state_posetion_ = curr_state_posetion_;
    //     return;
    // }


    // if(sum_state_delta_x > 1.5 || sum_state_delta_y > 1.5)
    // {
    //     pose.pose.position.x -= 0.1;
    //     pose.pose.position.y -= 0.1;
    // }

    last_state_posetion_ = curr_state_posetion_;

    
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();


    // if(sum_state_delta_x > 3 || sum_state_delta_y > 3)
    // {
    //     return;
    // }

    ros_path_.poses.push_back(pose);
}


// kitti
void LocalizationWrapper::SavePose(std::ofstream &ofs, const Eigen::Matrix4d &pose)
{
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 4; ++j)
        {
            ofs << pose(i, j);

            if(i == 2 && j == 3)
            {
                ofs << std::endl;
            }
            else
            {
                ofs << " ";
            }
        }
    }
}



// tum
// void LocalizationWrapper::SavePose(std::ofstream &ofs, const Eigen::Matrix4d &pose)
// {
//     file_state_ << std::fixed << std::setprecision(15) << ekf_ptr_->state_ptr_->timestamp << " "
//                 << ekf_ptr_->state_ptr_->p_wb_[0] << " " << ekf_ptr_->state_ptr_->p_wb_[1] << " "
//                 << ekf_ptr_->state_ptr_->p_wb_[2] << " " << q_GI.x() << " " << q_GI.y() << " " << q_GI.z() << " "
//                 << q_GI.w() << std::endl;
// }





