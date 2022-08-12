//
// Created by tao on 7/25/21.
//

#include "velodyne_converter.h"

#include <math.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kaist2bag {

VelodyneConverter::VelodyneConverter(const std::string &dataset_dir, const std::string &save_dir,
                                     const std::string &left_topic, const std::string& right_topic)
        : Converter(dataset_dir, save_dir), left_topic_(left_topic), right_topic_(right_topic) {
    left_bag_name_ = FilterSlash(left_topic_) + ".bag";
    right_bag_name_ = FilterSlash(right_topic_) + ".bag";
}

int VelodyneConverter::Convert() {
    CheckAndCreateSaveDir();

    boost::filesystem::path left_bag_file = boost::filesystem::path(save_dir_) / left_bag_name_;
    boost::filesystem::path right_bag_file = boost::filesystem::path(save_dir_) / right_bag_name_;

    const std::string left_stamp_file = dataset_dir_ + "/" + default_left_stamp_file;
    const std::string left_data_dir = dataset_dir_ + "/" + default_left_data_dir;
    const std::string right_stamp_file = dataset_dir_ + "/" + default_right_stamp_file;
    const std::string right_data_dir = dataset_dir_ + "/" + default_right_data_dir;

    ROS_INFO("saving %s", left_bag_file.c_str());
    ConvertLeft(left_stamp_file, left_data_dir, left_bag_file.string(), left_topic_, "left_velodyne");
    ROS_INFO("done saving %s", left_bag_file.c_str());
    ROS_INFO("saving %s", right_bag_file.c_str());
    ConvertLeft(right_stamp_file, right_data_dir, right_bag_file.string(), right_topic_, "right_velodyne");
    ROS_INFO("done saving %s", right_bag_file.c_str());

    return 0;
}

void VelodyneConverter::ConvertLeft(const std::string& stamp_file, const std::string& data_dir, const std::string& bag_file,
                                const std::string& topic, const std::string& frame_id) {
    rosbag::Bag bag(bag_file, rosbag::bagmode::Write);
    bag.setChunkThreshold(768*1024);
    bag.setCompression(rosbag::compression::BZ2);

    FILE* fp = fopen(stamp_file.c_str(), "r");
    int64_t stamp;
    std::vector<int64_t> all_stamps;
    while (fscanf(fp, "%ld\n", &stamp) == 1) {
        all_stamps.push_back(stamp);
    }
    fclose(fp);

    size_t total = all_stamps.size();
    for (size_t i = 0; i < all_stamps.size(); ++i) {
        std::string st = std::to_string(all_stamps[i]);
        std::string frame_file = data_dir + "/" + st + ".bin";
        ROS_INFO("converting %s\n", frame_file.c_str());
        if (!boost::filesystem::exists(frame_file)) {
            ROS_WARN("%s not exist\n", frame_file.c_str());
            continue;
        }
        std::ifstream file;
        file.open(frame_file, std::ios::in | std::ios::binary);
        pcl::PointCloud<PointXYZIR> pcl_cloud;
        pcl_cloud.clear();

        float angle;
        uint16_t ring;

        while (!file.eof()) {
            PointXYZIR point;
            file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
            angle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) / M_PI * 180 + 15;
            ring = round(angle / 2);
            point.ring = ring;
            pcl_cloud.points.push_back(point);
        }
        file.close();
        sensor_msgs::PointCloud2 cloud;
        pcl::toROSMsg(pcl_cloud, cloud);
        cloud.header.stamp.fromNSec(all_stamps[i]);
        cloud.header.frame_id = frame_id;
        bag.write(topic, cloud.header.stamp, cloud);
        ROS_INFO("bag write %s, %u points\n", topic.c_str(), cloud.height * cloud.width);
        ROS_INFO("done converting %s\n", frame_file.c_str());
        ROS_INFO("total %lu, already convert %lu, remain %lu\n", total, i + 1, total - i - 1);
    }
    bag.close();
}


void VelodyneConverter::ConvertRight(const std::string& stamp_file, const std::string& data_dir, const std::string& bag_file,
                                const std::string& topic, const std::string& frame_id) {
    rosbag::Bag bag(bag_file, rosbag::bagmode::Write);
    bag.setChunkThreshold(768*1024);
    bag.setCompression(rosbag::compression::BZ2);

    FILE* fp = fopen(stamp_file.c_str(), "r");
    int64_t stamp;
    std::vector<int64_t> all_stamps;
    while (fscanf(fp, "%ld\n", &stamp) == 1) {
        all_stamps.push_back(stamp);
    }
    fclose(fp);

    size_t total = all_stamps.size();
    for (size_t i = 0; i < all_stamps.size(); ++i) {
        std::string st = std::to_string(all_stamps[i]);
        std::string frame_file = data_dir + "/" + st + ".bin";
        ROS_INFO("converting %s\n", frame_file.c_str());
        if (!boost::filesystem::exists(frame_file)) {
            ROS_WARN("%s not exist\n", frame_file.c_str());
            continue;
        }
        std::ifstream file;
        file.open(frame_file, std::ios::in | std::ios::binary);
        pcl::PointCloud<PointXYZIR> pcl_cloud;
        pcl_cloud.clear();

        float angle;
        uint16_t ring;

        while (!file.eof()) {
            PointXYZIR point;
            file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
            angle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) / M_PI * 180 + 15;
            ring = round(angle / 2);
            point.ring = ring + 16;
            pcl_cloud.points.push_back(point);
        }
        file.close();
        sensor_msgs::PointCloud2 cloud;
        pcl::toROSMsg(pcl_cloud, cloud);
        cloud.header.stamp.fromNSec(all_stamps[i]);
        cloud.header.frame_id = frame_id;
        bag.write(topic, cloud.header.stamp, cloud);
        ROS_INFO("bag write %s, %u points\n", topic.c_str(), cloud.height * cloud.width);
        ROS_INFO("done converting %s\n", frame_file.c_str());
        ROS_INFO("total %lu, already convert %lu, remain %lu\n", total, i + 1, total - i - 1);
    }
    bag.close();
}

} // namespace kaist2bag