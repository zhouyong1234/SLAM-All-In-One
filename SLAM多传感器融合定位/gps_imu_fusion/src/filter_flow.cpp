//
// Created by meng on 2021/2/24.
//
#include "filter_flow.h"
#include "tool.h"
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
Eigen::Matrix4d Vector2Matrix(const Eigen::Vector3d& vec){
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3,1>(0,3) = vec;

    return matrix;
}

FilterFlow::FilterFlow(const std::string &work_space_path)
        : work_space_path_(work_space_path){

    std::string config_file_path = work_space_path_ + "/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    data_path_ = config_node["data_path"].as<std::string>();
    std::string filter_method = config_node["filter_method"].as<std::string>();
    if(filter_method == "ESKF"){
        filter_ptr_ = std::make_shared<ESKF>(config_node);
    }
    else if(filter_method == "EKF"){
        filter_ptr_ = std::make_shared<EKF>(config_node);
    }
    else if(filter_method == "ESKFQK"){
        filter_ptr_ = std::make_shared<ESKFQK>(config_node);
    }
    else {
        printf("no corres filter method");
        exit(-1);
    }
    printf("data_path: %s,  filter method: %s\n",data_path_.c_str(), filter_method.c_str());

}

// 读取GPS和IMU数据
bool FilterFlow::ReadData() {
    const std::string data_path = work_space_path_ + data_path_;

    if (imu_flow_ptr_->ReadIMUData(data_path, imu_data_buff_) &&
        gps_flow_ptr_->ReadGPSData(data_path, gps_data_buff_)
    ) {
        return false;
    }

    return false;
}

// IMIU和GPS时间差不能超过10ms
bool FilterFlow::ValidGPSAndIMUData() {
    curr_imu_data_ = imu_data_buff_.front();
    curr_gps_data_ = gps_data_buff_.front();

    double delta_time = curr_imu_data_.time - curr_gps_data_.time;

    // IMU来得晚了10ms，则GPS不满足条件
    if (delta_time > 0.01){
        gps_data_buff_.pop_front();
        return false;
    }

    if (delta_time < -0.01){
        imu_data_buff_.pop_front();
        return false;
    }

    imu_data_buff_.pop_front();
    gps_data_buff_.pop_front();

    return true;
}

bool FilterFlow::ValidIMUData() {
    curr_imu_data_ = imu_data_buff_.front();
    imu_data_buff_.front();

    return true;
}

bool FilterFlow::ValidGPSData() {
    curr_gps_data_ = gps_data_buff_.front();
    gps_data_buff_.pop_front();

    return true;
}

// 主函数流程
bool FilterFlow::Run() {
    ReadData();
    printf("data read successfully： imu size = %d, gps size = %d\n",(int)imu_data_buff_.size(),(int)gps_data_buff_.size());
    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()){
        if (!ValidGPSAndIMUData()){ // 确保初始的GPS和IMU时间不能相差超过10ms
            continue;
        } else{
            filter_ptr_->Init(curr_gps_data_, curr_imu_data_); // ESKF初始化
            break;
        }
    }

    std::ofstream gt_file(work_space_path_+"/data/gt.txt", std::ios::trunc);
    std::ofstream fused_file(work_space_path_+"/data/fused.txt", std::ios::trunc);
    std::ofstream measured_file(work_space_path_+"/data/measured.txt", std::ios::trunc);

    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()){
        curr_imu_data_ = imu_data_buff_.front();
        curr_gps_data_ = gps_data_buff_.front();
        if (curr_imu_data_.time < curr_gps_data_.time){ // IMU数据比当前GPS数据时间早，则使用IMU预测
            filter_ptr_->Predict(curr_imu_data_); // IMU预测
            imu_data_buff_.pop_front();
        } else{ // IMU数据时间比GPS晚一点
            // filter_ptr_->Predict(curr_imu_data_); // 预测
            // imu_data_buff_.pop_front();

            filter_ptr_->Correct(curr_gps_data_); // GPS数据观测

            SavePose(fused_file, filter_ptr_->GetPose());
            SavePose(measured_file,Vector2Matrix(curr_gps_data_.position_ned));

            SavePose(gt_file, Vector2Matrix(GPSFlow::LLA2NED(curr_gps_data_.true_position_lla)));
            gps_data_buff_.pop_front();
        }

        // if (use_observability_analysis_) {
        //     Eigen::Matrix<double, 15, 15> F;
        //     Eigen::Matrix<double, 3, 15> G;
        //     Eigen::Matrix<double, 3, 1> Y;
        //     filter_ptr_->GetFGY(F, G, Y);
        //     observability_analysis.SaveFG(F, G, Y, curr_gps_data_.time);
        // }
        // printf("1\n");
    }

    if (use_observability_analysis_) {
        observability_analysis.ComputeSOM();
        observability_analysis.ComputeObservability();
    }
    return true;
}

bool FilterFlow::TestRun() {
    ReadData();

    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()) {
        if (!ValidGPSAndIMUData()) {
            continue;
        } else {
            filter_ptr_->Init(curr_gps_data_, curr_imu_data_);
            std::cout << "\ntime: " << curr_gps_data_.time << std::endl;
            std::cout << "vel: " << filter_ptr_->GetVelocity().transpose() << std::endl;
            std::cout << "measure vel: " << curr_gps_data_.velocity.transpose() << std::endl;
            std::cout << "true vel: " << curr_gps_data_.true_velocity.transpose() << std::endl;
            std::cout << "time: " << curr_gps_data_.time << std::endl;
            break;
        }
    }

    std::ofstream gt_file(work_space_path_ + "/data/gt.txt", std::ios::trunc);
    std::ofstream fused_file(work_space_path_ + "/data/fused.txt", std::ios::trunc);
    std::ofstream measured_file(work_space_path_ + "/data/measured.txt", std::ios::trunc);

    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()) {
        curr_imu_data_ = imu_data_buff_.front();
        curr_gps_data_ = gps_data_buff_.front();
            filter_ptr_->Predict(curr_imu_data_);
            imu_data_buff_.pop_front();
            SavePose(fused_file, filter_ptr_->GetPose());
    }
    return true;
}

// 保存前3行4列
void FilterFlow::SavePose(std::ofstream &ofs, const Eigen::Matrix4d &pose) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i, j);

            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }
}

