//
// Created by tao on 7/24/21.
//

#include "imu_converter.h"

#include <rosbag/bag.h>
#include <irp_sen_msgs/imu.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <boost/filesystem.hpp>

namespace kaist2bag {

ImuConverter::ImuConverter(const std::string& dataset_dir, const std::string& save_dir,
                           const std::string& raw_topic)
        : Converter(dataset_dir, save_dir), raw_topic_(raw_topic)
{
    // irp_bag_name_ = FilterSlash(irp_topic_) + ".bag";
    raw_bag_name_ = FilterSlash(raw_topic_) + ".bag";
    // mag_bag_name_ = FilterSlash(mag_topic_) + ".bag";
}

int ImuConverter::Convert() {
    CheckAndCreateSaveDir();

    // boost::filesystem::path irp_bag_file = boost::filesystem::path(save_dir_) / irp_bag_name_;
    boost::filesystem::path raw_bag_file = boost::filesystem::path(save_dir_) / raw_bag_name_;
    // boost::filesystem::path mag_bag_file = boost::filesystem::path(save_dir_) / mag_bag_name_;
    // rosbag::Bag irp_bag(irp_bag_file.string(), rosbag::bagmode::Write);
    rosbag::Bag raw_bag(raw_bag_file.string(), rosbag::bagmode::Write);
    // rosbag::Bag mag_bag(mag_bag_file.string(), rosbag::bagmode::Write);
    // ROS_INFO("saving %s", irp_bag_file.c_str());
    ROS_INFO("saving %s", raw_bag_file.c_str());
    // ROS_INFO("saving %s", mag_bag_file.c_str());

    const std::string data_file = dataset_dir_ + "/" + default_data_file;
    FILE* fp = fopen(data_file.c_str(), "r");
    // irp_sen_msgs::imu irp_imu;
    sensor_msgs::Imu sensor_msgs_imu;
    // sensor_msgs::MagneticField sensor_msgs_mag;

    

    int64_t stamp;
    double q_x,q_y,q_z,q_w,x,y,z,g_x,g_y,g_z,a_x,a_y,a_z,m_x,m_y,m_z;
    while (fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                  &stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z) == 17) {
        // irp_imu.header.stamp.fromNSec(stamp);
        // irp_imu.header.frame_id = "imu";
        // irp_imu.quaternion_data.x = q_x;
        // irp_imu.quaternion_data.y = q_y;
        // irp_imu.quaternion_data.z = q_z;
        // irp_imu.quaternion_data.w = q_w;
        // irp_imu.eular_data.x = x;
        // irp_imu.eular_data.y = y;
        // irp_imu.eular_data.z = z;
        // irp_imu.gyro_data.x = g_x;
        // irp_imu.gyro_data.y = g_y;
        // irp_imu.gyro_data.z = g_z;
        // irp_imu.acceleration_data.x = a_x;
        // irp_imu.acceleration_data.y = a_y;
        // irp_imu.acceleration_data.z = a_z;
        // irp_imu.magneticfield_data.x = m_x;
        // irp_imu.magneticfield_data.y = m_y;
        // irp_imu.magneticfield_data.z = m_z;

        sensor_msgs_imu.header.stamp.fromNSec(stamp);
        sensor_msgs_imu.header.frame_id = "imu";
        sensor_msgs_imu.orientation.x = q_x;
        sensor_msgs_imu.orientation.y = q_y;
        sensor_msgs_imu.orientation.z = q_z;
        sensor_msgs_imu.orientation.w = q_w;
        sensor_msgs_imu.angular_velocity.x = g_x;
        sensor_msgs_imu.angular_velocity.y = g_y;
        sensor_msgs_imu.angular_velocity.z = g_z;
        sensor_msgs_imu.linear_acceleration.x = a_x;
        sensor_msgs_imu.linear_acceleration.y = a_y;
        sensor_msgs_imu.linear_acceleration.z = a_z;
        sensor_msgs_imu.orientation_covariance[0] = 3;
        sensor_msgs_imu.orientation_covariance[4] = 3;
        sensor_msgs_imu.orientation_covariance[8] = 3;
        sensor_msgs_imu.angular_velocity_covariance[0] = 3;
        sensor_msgs_imu.angular_velocity_covariance[4] = 3;
        sensor_msgs_imu.angular_velocity_covariance[8] = 3;
        sensor_msgs_imu.linear_acceleration_covariance[0] = 3;
        sensor_msgs_imu.linear_acceleration_covariance[4] = 3;
        sensor_msgs_imu.linear_acceleration_covariance[8] = 3;

        // sensor_msgs_mag.header.stamp.fromNSec(stamp);
        // sensor_msgs_mag.header.frame_id = "imu";
        // sensor_msgs_mag.magnetic_field.x = m_x;
        // sensor_msgs_mag.magnetic_field.y = m_y;
        // sensor_msgs_mag.magnetic_field.z = m_z;

        // irp_bag.write(irp_topic_, irp_imu.header.stamp, irp_imu);
        raw_bag.write(raw_topic_, sensor_msgs_imu.header.stamp, sensor_msgs_imu);
        // mag_bag.write(mag_topic_, sensor_msgs_mag.header.stamp, sensor_msgs_mag);
    }
    // irp_bag.close();
    raw_bag.close();
    // mag_bag.close();
    fclose(fp);
    // ROS_INFO("done saving %s", irp_bag_file.c_str());
    ROS_INFO("done saving %s", raw_bag_file.c_str());
    // ROS_INFO("done saving %s", mag_bag_file.c_str());

    return 0;
}

} // namespace kaist2bag