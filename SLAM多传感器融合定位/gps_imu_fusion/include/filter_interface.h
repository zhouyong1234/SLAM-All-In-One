#ifndef FILTER_INTERFACE_H_
#define FILTER_INTERFACE_H_
#include "imu_data.h"
#include "gps_data.h"

class  FilterInterface{
public:
    virtual ~FilterInterface()=default;
    /*!
     * 用于滤波器的初始化，设置初始位姿，初始速度
     * @param curr_gps_data 与imu时间同步的gps数据
     * @param curr_imu_data 与gps时间同步的imu数据
     * @return
     */
    virtual bool Init(const GPSData &curr_gps_data, const IMUData &curr_imu_data) = 0;

    /*!
     * 滤波器的预测，对应卡尔曼滤波器的前两个公式
     * @param curr_imu_data
     * @return
     */
    virtual bool Predict(const IMUData &curr_imu_data) = 0;

    /*!
     * 滤波器的矫正，对应卡尔曼滤波器的后三个公式
     * @param curr_gps_data
     * @return
     */
    virtual bool Correct(const GPSData &curr_gps_data) = 0;

    virtual Eigen::Matrix4d GetPose() const = 0;

    virtual Eigen::Vector3d GetVelocity() = 0;
    
    // void GetFGY(TypeMatrixF& F,TypeMatrixG& G, TypeVectorY & Y);
};

#endif