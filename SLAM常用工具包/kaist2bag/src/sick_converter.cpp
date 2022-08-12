//
// Created by tao on 7/28/21.
//

#include "sick_converter.h"

#include <rosbag/bag.h>
#include <sensor_msgs/LaserScan.h>
#include <irp_sen_msgs/LaserScanArray.h>
#include <boost/filesystem.hpp>

namespace kaist2bag {

SickConverter::SickConverter(const std::string &dataset_dir, const std::string &save_dir,
                                     const std::string &back_topic, const std::string& middle_topic)
        : Converter(dataset_dir, save_dir), back_topic_(back_topic), middle_topic_(middle_topic) {
    back_bag_name_ = FilterSlash(back_topic_) + ".bag";
    middle_bag_name_ = FilterSlash(middle_topic_) + ".bag";
}

int SickConverter::Convert() {
    CheckAndCreateSaveDir();

    boost::filesystem::path back_bag_file = boost::filesystem::path(save_dir_) / back_bag_name_;
    boost::filesystem::path middle_bag_file = boost::filesystem::path(save_dir_) / middle_bag_name_;

    const std::string back_stamp_file = dataset_dir_ + "/" + default_back_stamp_file;
    const std::string back_data_dir = dataset_dir_ + "/" + default_back_data_dir;
    const std::string middle_stamp_file = dataset_dir_ + "/" + default_middle_stamp_file;
    const std::string middle_data_dir = dataset_dir_ + "/" + default_middle_data_dir;

    ROS_INFO("saving %s", back_bag_file.c_str());
    Convert(back_stamp_file, back_data_dir, back_bag_file.string(), back_topic_, "back_sick");
    ROS_INFO("done saving %s", back_bag_file.c_str());
    ROS_INFO("saving %s", middle_bag_file.c_str());
    Convert(middle_stamp_file, middle_data_dir, middle_bag_file.string(), middle_topic_, "middle_sick");
    ROS_INFO("done saving %s", middle_bag_file.c_str());

    return 0;
}

void SickConverter::Convert(const std::string& stamp_file, const std::string& data_dir, const std::string& bag_file,
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
        irp_sen_msgs::LaserScanArray scan_array;
        sensor_msgs::LaserScan scan;
        while (!file.eof()) {
            float range;
            float intensity;
            file.read(reinterpret_cast<char *>(&range), sizeof(float));
            file.read(reinterpret_cast<char *>(&intensity), sizeof(float));
            scan.ranges.push_back(range);
            scan.intensities.push_back((intensity));
        }
        file.close();
        scan.header.stamp.fromNSec(all_stamps[i]);
        scan.header.frame_id = frame_id;
        scan.angle_min = -1.65806281567;
        scan.angle_max = -1.65806281567;
        scan.angle_increment = 0.0116355288774;
        scan.time_increment = 0.0;
        scan.range_min = 0.0;
        scan.range_max = 81.0;
        scan_array.LaserScans.push_back(scan);
        scan_array.size = scan_array.LaserScans.size();

        scan_array.header.stamp.fromNSec(all_stamps[i]);
        scan_array.header.frame_id = "back_sick";        
        bag.write(topic, scan_array.header.stamp, scan_array);
        ROS_INFO("done converting %s\n", frame_file.c_str());
        ROS_INFO("total %lu, already convert %lu, remain %lu\n", total, i + 1, total - i - 1);
    }
    bag.close();
}

} // namespace kaist2bag