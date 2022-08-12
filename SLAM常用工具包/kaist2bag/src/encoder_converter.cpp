//
// Created by tao on 7/21/21.
//

#include "encoder_converter.h"

#include <rosbag/bag.h>
#include <irp_sen_msgs/encoder.h>
#include <sensor_msgs/JointState.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

namespace kaist2bag {


EncoderConverter::EncoderConverter(const std::string &dataset_dir, const std::string &save_dir,
                                   const std::string &irp_topic, const std::string &raw_topic)
                                   : Converter(dataset_dir, save_dir), irp_topic_(irp_topic), raw_topic_(raw_topic) {
    irp_bag_name_ = FilterSlash(irp_topic_) + ".bag";
    raw_bag_name_ = FilterSlash(raw_topic_) + ".bag";
}

int EncoderConverter::Convert() {
    CheckAndCreateSaveDir();

    boost::filesystem::path irp_bag_file = boost::filesystem::path(save_dir_) / irp_bag_name_;
    boost::filesystem::path raw_bag_file = boost::filesystem::path(save_dir_) / raw_bag_name_;
    rosbag::Bag irp_bag(irp_bag_file.string(), rosbag::bagmode::Write);
    rosbag::Bag raw_bag(raw_bag_file.string(), rosbag::bagmode::Write);
    ROS_INFO("saving %s", irp_bag_file.c_str());
    ROS_INFO("saving %s", raw_bag_file.c_str());

    sensor_msgs::JointState joint_data;
    joint_data.name.push_back("front_left_wheel_joint");
    joint_data.name.push_back("front_right_wheel_joint");
    joint_data.name.push_back("back_left_wheel_joint");
    joint_data.name.push_back("back_right_wheel_joint");
    joint_data.position.push_back(0.0);
    joint_data.position.push_back(0.0);
    joint_data.position.push_back(0.0);
    joint_data.position.push_back(0.0);
    joint_data.velocity.push_back(0.0);
    joint_data.velocity.push_back(0.0);
    joint_data.velocity.push_back(0.0);
    joint_data.velocity.push_back(0.0);
    bool has_previous = false;
    int64_t pre_stamp, pre_left_count, pre_right_count;

    int encoder_resolution_ = 0;
    double encoder_left_diameter_ = 0.0;
    double encoder_right_diameter_ = 0.0;
    double encoder_wheel_base_ = 0.0;
    const std::string calib_file = dataset_dir_ + "/" + default_calib_file;
    std::ifstream file(calib_file.c_str());
    std::string str;
    while (std::getline(file, str)) {
        std::vector<std::string> strs;
        boost::split(strs, str, boost::is_any_of(" "));
        if(!strs[1].compare("resolution:")){
            encoder_resolution_ = std::stoi(strs[2]);
        }
        if(!strs[1].compare("left")){
            encoder_left_diameter_ = std::stod(strs[4]);
        }
        if(!strs[1].compare("right")){
            encoder_right_diameter_ = std::stod(strs[4]);
        }
        if(!strs[1].compare("wheel")){
            encoder_wheel_base_ = std::stod(strs[3]);
        }
    }

    const std::string data_file = dataset_dir_ + "/" + default_data_file;
    FILE* fp = fopen(data_file.c_str(), "r");
    irp_sen_msgs::encoder encoder_data;
    int64_t stamp, left_count, right_count;
    while (fscanf(fp, "%ld,%ld,%ld\n", &stamp, &left_count, &right_count) == 3) {
        encoder_data.header.stamp.fromNSec(stamp);
        encoder_data.header.frame_id = "encoder";
        encoder_data.left_count = left_count;
        encoder_data.right_count = right_count;
        irp_bag.write(irp_topic_, encoder_data.header.stamp, encoder_data);

        if(has_previous) {

            double stamp_diff = 1e-9 * static_cast<double>(stamp - pre_stamp);
            int64_t d_left_cnt = left_count - pre_left_count;
            int64_t d_right_cnt = right_count - pre_right_count;
            double degl = ((double)d_left_cnt/(double)encoder_resolution_) * 2 * M_PI;
            double degr = ((double)d_right_cnt/(double)encoder_resolution_) * 2 * M_PI;
            double wl = degl / stamp_diff;
            double wr = degr / stamp_diff;
            
            joint_data.header.stamp.fromNSec(stamp);
            joint_data.header.frame_id = "encoder";
            joint_data.position[0] += degl;
            joint_data.position[1] += degr;
            joint_data.position[2] += degl;
            joint_data.position[3] += degr;
            joint_data.velocity[0] = wl;
            joint_data.velocity[1] = wr;
            joint_data.velocity[2] = wl;
            joint_data.velocity[3] = wr;
            raw_bag.write(raw_topic_, joint_data.header.stamp, joint_data);

        }
        has_previous = true;
        pre_stamp = stamp;
        pre_left_count = left_count;
        pre_right_count = right_count;

    }
    irp_bag.close();
    raw_bag.close();
    fclose(fp);
    ROS_INFO("done saving %s", irp_bag_file.c_str());
    ROS_INFO("done saving %s", raw_bag_file.c_str());

    return 0;
}
} // namespace kaist2bag

